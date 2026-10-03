"""Deterministic policy/state for the bounded agent-facing robot CLI."""

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

from soarm101_motion.workspace import WorkspaceCalibration

AGENT_AUTHORITY_SCHEMA_VERSION = 1
AGENT_POSE_PREFIX = "agent_"
AGENT_CAMERA_NAMES = ("overhead", "wrist")

DEFAULT_AUTHORITY_MINUTES = 60.0
MAX_AUTHORITY_MINUTES = 480.0

AGENT_JOG_HEIGHT_THRESHOLD_M = 0.100
AGENT_JOG_HIGH_MAX_DISTANCE_M = 0.050
AGENT_JOG_LOW_MAX_DISTANCE_M = 0.010
AGENT_JOG_MINIMUM_TARGET_HEIGHT_M = 0.0


def default_agent_authority_path() -> Path:
    override = os.environ.get("SOARM101_AGENT_AUTHORITY_PATH", "").strip()
    if override:
        return Path(override).expanduser()
    return Path.home() / ".local" / "state" / "soarm101" / "agent" / "authority.json"


@dataclass(frozen=True)
class AgentAuthority:
    """Time-bounded human authorization for one robot/calibration identity."""

    schema_version: int
    robot_id: str
    calibration_id: str
    issued_at: float
    expires_at: float

    def validated(self) -> "AgentAuthority":
        if int(self.schema_version) != AGENT_AUTHORITY_SCHEMA_VERSION:
            raise ValueError(
                f"unsupported agent authority schema {self.schema_version}; "
                f"expected {AGENT_AUTHORITY_SCHEMA_VERSION}"
            )
        robot_id = str(self.robot_id).strip()
        calibration_id = str(self.calibration_id).strip()
        issued_at = float(self.issued_at)
        expires_at = float(self.expires_at)
        if not robot_id or not calibration_id:
            raise ValueError("agent authority robot/calibration identity cannot be empty")
        if not all(math.isfinite(value) for value in (issued_at, expires_at)):
            raise ValueError("agent authority timestamps must be finite")
        if expires_at <= issued_at:
            raise ValueError("agent authority expiry must be after issuance")
        return AgentAuthority(
            schema_version=AGENT_AUTHORITY_SCHEMA_VERSION,
            robot_id=robot_id,
            calibration_id=calibration_id,
            issued_at=issued_at,
            expires_at=expires_at,
        )

    def active(self, *, now: float | None = None) -> bool:
        instant = time.time() if now is None else float(now)
        return instant < self.expires_at

    def status_payload(self, *, now: float | None = None) -> dict[str, object]:
        instant = time.time() if now is None else float(now)
        return {
            "armed": instant < self.expires_at,
            "robot_id": self.robot_id,
            "calibration_id": self.calibration_id,
            "issued_at": self.issued_at,
            "expires_at": self.expires_at,
            "remaining_seconds": max(0.0, self.expires_at - instant),
        }


class AgentAuthorityStore:
    """Atomic machine-local storage for the human-authorized agent motion lease."""

    def __init__(self, path: str | Path | None = None) -> None:
        self.path = (
            Path(path).expanduser()
            if path is not None
            else default_agent_authority_path()
        )

    def load(self) -> AgentAuthority | None:
        if not self.path.is_file():
            return None
        payload = json.loads(self.path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("agent authority root must be a JSON object")
        return AgentAuthority(**payload).validated()

    def issue(
        self,
        *,
        robot_id: str,
        calibration_id: str,
        minutes: float = DEFAULT_AUTHORITY_MINUTES,
        now: float | None = None,
    ) -> AgentAuthority:
        duration_minutes = float(minutes)
        if (
            not math.isfinite(duration_minutes)
            or duration_minutes <= 0.0
            or duration_minutes > MAX_AUTHORITY_MINUTES
        ):
            raise ValueError(
                f"agent authority minutes must be within (0, {MAX_AUTHORITY_MINUTES:g}]"
            )
        issued_at = time.time() if now is None else float(now)
        authority = AgentAuthority(
            schema_version=AGENT_AUTHORITY_SCHEMA_VERSION,
            robot_id=robot_id,
            calibration_id=calibration_id,
            issued_at=issued_at,
            expires_at=issued_at + duration_minutes * 60.0,
        ).validated()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(json.dumps(asdict(authority), indent=2) + "\n", encoding="utf-8")
        os.chmod(temporary, 0o600)
        os.replace(temporary, self.path)
        return authority

    def clear(self) -> None:
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass

    def status(self, *, now: float | None = None) -> dict[str, object]:
        authority = self.load()
        if authority is None:
            return {
                "armed": False,
                "robot_id": None,
                "calibration_id": None,
                "issued_at": None,
                "expires_at": None,
                "remaining_seconds": 0.0,
            }
        return authority.status_payload(now=now)

    def require(
        self,
        *,
        robot_id: str,
        calibration_id: str,
        now: float | None = None,
    ) -> AgentAuthority:
        authority = self.load()
        if authority is None:
            raise PermissionError(
                "agent motion is not armed; a human must run 'soarm101 agent arm' first"
            )
        instant = time.time() if now is None else float(now)
        if not authority.active(now=instant):
            self.clear()
            raise PermissionError(
                "agent motion authority has expired; a human must re-arm the session"
            )
        if authority.robot_id != str(robot_id):
            raise PermissionError(
                f"agent authority is for robot {authority.robot_id!r}, "
                f"not {str(robot_id)!r}"
            )
        if authority.calibration_id != str(calibration_id):
            raise PermissionError(
                "agent authority calibration no longer matches the active robot calibration; "
                "a human must review and re-arm"
            )
        return authority


@dataclass(frozen=True)
class AgentJogDecision:
    """Result of the physical-height policy applied to one model-frame jog."""

    current_height_m: float
    target_height_m: float
    requested_distance_m: float
    maximum_distance_m: float

    def to_payload(self) -> dict[str, float]:
        return {
            "current_height_mm": self.current_height_m * 1000.0,
            "target_height_mm": self.target_height_m * 1000.0,
            "requested_distance_mm": self.requested_distance_m * 1000.0,
            "maximum_distance_mm": self.maximum_distance_m * 1000.0,
        }


def evaluate_agent_jog(
    workspace: WorkspaceCalibration,
    *,
    active_calibration_id: str,
    current_model_position_m: Sequence[float],
    delta_model_m: Sequence[float],
) -> AgentJogDecision:
    """Apply the bounded agent-jog policy using measured physical workspace height.

    The workspace transform is used only as safety evidence to measure physical
    displacement and height. The actual command remains a normal model-frame SDK jog.
    """

    calibration = workspace.validated()
    if calibration.arm_calibration_id != str(active_calibration_id):
        raise PermissionError(
            "workspace calibration does not match the active motor calibration"
        )
    current_model = np.asarray(tuple(float(v) for v in current_model_position_m), dtype=float)
    delta_model = np.asarray(tuple(float(v) for v in delta_model_m), dtype=float)
    if current_model.shape != (3,) or delta_model.shape != (3,):
        raise ValueError("agent jog positions must contain exactly three values")
    if not np.all(np.isfinite(current_model)) or not np.all(np.isfinite(delta_model)):
        raise ValueError("agent jog positions must be finite")

    target_model = current_model + delta_model
    current_physical = calibration.physical_position_from_model(current_model)
    target_physical = calibration.physical_position_from_model(target_model)
    current_height = float(current_physical[2])
    target_height = float(target_physical[2])
    requested_distance = float(np.linalg.norm(target_physical - current_physical))
    maximum_distance = (
        AGENT_JOG_HIGH_MAX_DISTANCE_M
        if current_height > AGENT_JOG_HEIGHT_THRESHOLD_M
        else AGENT_JOG_LOW_MAX_DISTANCE_M
    )

    if requested_distance <= 1e-9:
        raise ValueError("agent jog must request a non-zero translation")
    if requested_distance > maximum_distance + 1e-9:
        raise PermissionError(
            f"agent jog physical distance {requested_distance * 1000.0:.1f} mm exceeds "
            f"the {maximum_distance * 1000.0:.1f} mm limit at current physical height "
            f"{current_height * 1000.0:.1f} mm"
        )
    if target_height < AGENT_JOG_MINIMUM_TARGET_HEIGHT_M - 1e-9:
        raise PermissionError(
            f"agent jog target physical height {target_height * 1000.0:.1f} mm "
            "would cross the calibrated ground plane"
        )

    return AgentJogDecision(
        current_height_m=current_height,
        target_height_m=target_height,
        requested_distance_m=requested_distance,
        maximum_distance_m=maximum_distance,
    )
