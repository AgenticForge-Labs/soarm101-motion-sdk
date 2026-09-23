"""Persistent named poses shared by the GUI, sequences, and higher-level consumers."""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import TYPE_CHECKING, Any

from soarm101_motion.constants import ARM_JOINTS

if TYPE_CHECKING:
    from soarm101_motion.arm import SOARM101

HOME_POSE_NAME = "home"
REST_POSE_NAME = "rest"


def _utc_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass(frozen=True)
class SavedPose:
    """Measured robot pose suitable for later joint or Cartesian playback."""

    joints: Mapping[str, float]
    gripper: float
    tcp_xyz_rpy: tuple[float, float, float, float, float, float]
    source: str = "follower"
    source_robot_id: str | None = None
    source_calibration_id: str | None = None
    target_robot_id: str | None = None
    target_calibration_id: str | None = None
    created_at: str = field(default_factory=_utc_timestamp)

    def __post_init__(self) -> None:
        joints = {name: float(value) for name, value in self.joints.items()}
        missing = set(ARM_JOINTS) - set(joints)
        extra = set(joints) - set(ARM_JOINTS)
        if missing or extra:
            raise ValueError(
                f"saved pose joints must match {tuple(ARM_JOINTS)}; "
                f"missing={sorted(missing)}, extra={sorted(extra)}"
            )
        if not all(math.isfinite(value) for value in joints.values()):
            raise ValueError("saved pose joint values must be finite")
        gripper = float(self.gripper)
        if not math.isfinite(gripper) or not 0.0 <= gripper <= 1.0:
            raise ValueError("saved pose gripper must be finite and within [0, 1]")
        tcp = tuple(float(value) for value in self.tcp_xyz_rpy)
        if len(tcp) != 6 or not all(math.isfinite(value) for value in tcp):
            raise ValueError("saved pose TCP must contain six finite xyz/rpy values")
        source = str(self.source).strip() or "unknown"
        for key in (
            "source_robot_id",
            "source_calibration_id",
            "target_robot_id",
            "target_calibration_id",
        ):
            value = getattr(self, key)
            object.__setattr__(
                self,
                key,
                None if value is None or not str(value).strip() else str(value).strip(),
            )
        object.__setattr__(self, "joints", joints)
        object.__setattr__(self, "gripper", gripper)
        object.__setattr__(self, "tcp_xyz_rpy", tcp)
        object.__setattr__(self, "source", source)

    @classmethod
    def capture(cls, arm: "SOARM101", *, source: str = "follower") -> "SavedPose":
        """Capture one measured arm snapshot.

        Joint positions are read once and reused for forward kinematics so the saved
        TCP corresponds to the same measured joint sample. This matters especially
        during live teleoperation, when the follower may be moving between reads.
        """

        joints = dict(arm.get_joint_positions().positions)
        gripper = float(arm.tool.get_position())
        tcp_pose = arm.model.forward(joints, tcp=arm.active_tcp)
        tcp = tuple(float(value) for value in tcp_pose.xyz_rpy())
        return cls(
            joints=joints,
            gripper=gripper,
            tcp_xyz_rpy=tcp,
            source=source,
            source_robot_id=arm.config.robot_id,
            source_calibration_id=arm.calibration_id,
        )

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "SavedPose":
        return cls(
            joints=dict(data["joints"]),
            gripper=float(data["gripper"]),
            tcp_xyz_rpy=tuple(float(value) for value in data["tcp_xyz_rpy"]),  # type: ignore[arg-type]
            source=str(data.get("source", "unknown")),
            source_robot_id=(
                None if data.get("source_robot_id") is None else str(data["source_robot_id"])
            ),
            source_calibration_id=(
                None
                if data.get("source_calibration_id") is None
                else str(data["source_calibration_id"])
            ),
            target_robot_id=(
                None if data.get("target_robot_id") is None else str(data["target_robot_id"])
            ),
            target_calibration_id=(
                None
                if data.get("target_calibration_id") is None
                else str(data["target_calibration_id"])
            ),
            created_at=str(data.get("created_at") or _utc_timestamp()),
        )

    def to_mapping(self) -> dict[str, Any]:
        return {
            "joints": dict(self.joints),
            "gripper": self.gripper,
            "tcp_xyz_rpy": list(self.tcp_xyz_rpy),
            "source": self.source,
            "source_robot_id": self.source_robot_id,
            "source_calibration_id": self.source_calibration_id,
            "target_robot_id": self.target_robot_id,
            "target_calibration_id": self.target_calibration_id,
            "created_at": self.created_at,
        }


def default_pose_library_path(robot_id: str) -> Path:
    return Path.home() / ".config" / "soarm101" / "poses" / f"{robot_id}.json"


class PoseLibrary:
    """Small human-readable named-pose store.

    Home, rest, and taught points use the same representation so later sequence
    and Puppeteer layers can consume them without depending on the GUI.
    """

    schema_version = 1

    def __init__(self, robot_id: str, *, path: str | Path | None = None) -> None:
        self.robot_id = str(robot_id).strip() or "so101"
        self.path = Path(path).expanduser() if path is not None else default_pose_library_path(self.robot_id)
        self._poses: dict[str, SavedPose] = {}
        self.reload()

    def reload(self) -> None:
        self._poses = {}
        if not self.path.is_file():
            return
        data = json.loads(self.path.read_text(encoding="utf-8"))
        raw_poses = data.get("poses", {})
        self._poses = {
            str(name): SavedPose.from_mapping(value)
            for name, value in raw_poses.items()
        }

    def _write(self) -> Path:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema_version": self.schema_version,
            "robot_id": self.robot_id,
            "poses": {
                name: pose.to_mapping()
                for name, pose in sorted(self._poses.items())
            },
        }
        self.path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        return self.path

    def names(self) -> tuple[str, ...]:
        return tuple(sorted(self._poses))

    def get(self, name: str) -> SavedPose | None:
        return self._poses.get(str(name).strip())

    def require(self, name: str) -> SavedPose:
        key = str(name).strip()
        pose = self.get(key)
        if pose is None:
            raise KeyError(f"saved pose {key!r} does not exist")
        return pose

    def save(self, name: str, pose: SavedPose) -> Path:
        key = str(name).strip()
        if not key:
            raise ValueError("pose name must not be empty")
        self._poses[key] = pose
        return self._write()

    def delete(self, name: str) -> Path:
        key = str(name).strip()
        if key not in self._poses:
            raise KeyError(key)
        del self._poses[key]
        return self._write()
