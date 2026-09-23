"""Calibration storage and LeRobot-compatible conversion."""

from __future__ import annotations

import hashlib
import json
import os
from dataclasses import asdict, dataclass
from math import pi
from pathlib import Path
from typing import Any, Mapping

from soarm101_motion.constants import ALL_MOTORS, ENCODER_MAX, STOCK_GRIPPER
from soarm101_motion.exceptions import CalibrationError, SafetyViolationError

_LEROBOT_TO_SDK = {"gripper": STOCK_GRIPPER}
_SDK_TO_LEROBOT = {STOCK_GRIPPER: "gripper"}


@dataclass(frozen=True)
class MotorCalibration:
    motor_id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "MotorCalibration":
        return cls(
            motor_id=int(data.get("motor_id", data.get("id"))),
            drive_mode=int(data.get("drive_mode", 0)),
            homing_offset=int(data.get("homing_offset", 0)),
            range_min=int(data.get("range_min", 0)),
            range_max=int(data.get("range_max", ENCODER_MAX)),
        )

    def validate(self) -> None:
        if self.range_min >= self.range_max:
            raise CalibrationError(
                f"motor {self.motor_id} has invalid range {self.range_min}..{self.range_max}"
            )
        if self.drive_mode not in (0, 1):
            raise CalibrationError(f"motor {self.motor_id} has invalid drive_mode={self.drive_mode}")

    @property
    def center_raw(self) -> float:
        """Calibrated joint zero in raw encoder coordinates.

        LeRobot degree normalization defines zero from the midpoint of the recorded
        calibrated range. Native mechanical-stop calibration produces symmetric
        limits around the Feetech half-turn reference, so its midpoint remains 2047.
        """

        return (float(self.range_min) + float(self.range_max)) / 2.0

    @property
    def radians_limits(self) -> tuple[float, float]:
        scale = 2.0 * pi / ENCODER_MAX
        center = self.center_raw
        return (self.range_min - center) * scale, (self.range_max - center) * scale

    def raw_to_radians(self, raw: int) -> float:
        return (float(raw) - self.center_raw) * 2.0 * pi / ENCODER_MAX

    def radians_to_raw(self, radians: float) -> int:
        raw = int(
            round(float(radians) * ENCODER_MAX / (2.0 * pi) + self.center_raw)
        )
        if not self.range_min <= raw <= self.range_max:
            raise SafetyViolationError(
                f"joint target maps to raw position {raw}, outside calibrated range "
                f"[{self.range_min}, {self.range_max}] for motor {self.motor_id}"
            )
        return raw

    def raw_to_normalized(self, raw: int) -> float:
        bounded = min(self.range_max, max(self.range_min, int(raw)))
        value = (bounded - self.range_min) / (self.range_max - self.range_min)
        return 1.0 - value if self.drive_mode else value

    def normalized_to_raw(self, normalized: float) -> int:
        value = float(normalized)
        if not 0.0 <= value <= 1.0:
            raise SafetyViolationError(f"normalized tool position {value} is outside [0, 1]")
        value = 1.0 - value if self.drive_mode else value
        return int(round(self.range_min + value * (self.range_max - self.range_min)))


@dataclass(frozen=True)
class SO101Calibration:
    motors: Mapping[str, MotorCalibration]
    schema_version: int = 1
    source: str = "soarm101-motion-sdk"

    def validate(self, *, require_all: bool = True) -> None:
        if require_all:
            missing = set(ALL_MOTORS) - set(self.motors)
            if missing:
                raise CalibrationError(f"calibration is missing motors: {sorted(missing)}")
        for name, calibration in self.motors.items():
            if name not in ALL_MOTORS:
                raise CalibrationError(f"unknown calibrated motor: {name}")
            calibration.validate()

    def canonical_motor_mapping(self) -> dict[str, dict[str, int]]:
        """Return only physical calibration fields in deterministic order."""

        return {
            name: {
                "motor_id": int(self.motors[name].motor_id),
                "drive_mode": int(self.motors[name].drive_mode),
                "homing_offset": int(self.motors[name].homing_offset),
                "range_min": int(self.motors[name].range_min),
                "range_max": int(self.motors[name].range_max),
            }
            for name in sorted(self.motors)
        }

    @property
    def fingerprint(self) -> str:
        """SHA-256 identity for the physical calibration, independent of file metadata."""

        payload = json.dumps(
            self.canonical_motor_mapping(),
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        return hashlib.sha256(payload).hexdigest()

    @property
    def calibration_id(self) -> str:
        return f"sha256:{self.fingerprint}"

    @property
    def is_factory_range(self) -> bool:
        return all(
            calibration.range_min == 0 and calibration.range_max == ENCODER_MAX
            for calibration in self.motors.values()
        )

    @property
    def uncalibrated_motors(self) -> tuple[str, ...]:
        """Return motors whose EEPROM still has an unrestricted factory range."""

        return tuple(
            name
            for name, calibration in self.motors.items()
            if calibration.range_min == 0 and calibration.range_max == ENCODER_MAX
        )

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any], *, source: str = "unknown") -> "SO101Calibration":
        if "motors" in data:
            raw_motors = data["motors"]
            schema_version = int(data.get("schema_version", 1))
            source = str(data.get("source", source))
        else:
            raw_motors = data
            schema_version = 1
            source = "lerobot"
        motors: dict[str, MotorCalibration] = {}
        for raw_name, raw_calibration in raw_motors.items():
            name = _LEROBOT_TO_SDK.get(str(raw_name), str(raw_name))
            motors[name] = MotorCalibration.from_mapping(raw_calibration)
        result = cls(motors=motors, schema_version=schema_version, source=source)
        result.validate(require_all=False)
        return result

    @classmethod
    def load(cls, path: str | Path) -> "SO101Calibration":
        path = Path(path).expanduser()
        try:
            with path.open(encoding="utf-8") as handle:
                data = json.load(handle)
        except (OSError, json.JSONDecodeError) as exc:
            raise CalibrationError(f"could not load calibration from {path}") from exc
        return cls.from_mapping(data, source=str(path))

    def save(self, path: str | Path) -> Path:
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema_version": self.schema_version,
            "source": self.source,
            "calibration_id": self.calibration_id,
            "motors": {name: asdict(calibration) for name, calibration in self.motors.items()},
        }
        path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        return path

    def save_lerobot(self, path: str | Path) -> Path:
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        payload: dict[str, dict[str, int]] = {}
        for name, calibration in self.motors.items():
            key = _SDK_TO_LEROBOT.get(name, name)
            payload[key] = {
                "id": calibration.motor_id,
                "drive_mode": calibration.drive_mode,
                "homing_offset": calibration.homing_offset,
                "range_min": calibration.range_min,
                "range_max": calibration.range_max,
            }
        path.write_text(json.dumps(payload, indent=4) + "\n", encoding="utf-8")
        return path


def default_calibration_path(robot_id: str) -> Path:
    return Path.home() / ".config" / "soarm101" / "calibration" / f"{robot_id}.json"


def default_calibration_history_dir(robot_id: str) -> Path:
    return (
        Path.home()
        / ".config"
        / "soarm101"
        / "calibration"
        / "history"
        / (str(robot_id).strip() or "so101")
    )


def calibration_history_path(robot_id: str, calibration: SO101Calibration) -> Path:
    return default_calibration_history_dir(robot_id) / f"{calibration.fingerprint}.json"


def archive_calibration(
    calibration: SO101Calibration,
    *,
    robot_id: str,
    history_dir: str | Path | None = None,
) -> Path:
    """Persist one immutable calibration snapshot keyed by its content fingerprint."""

    calibration.validate()
    root = (
        Path(history_dir).expanduser()
        if history_dir is not None
        else default_calibration_history_dir(robot_id)
    )
    path = root / f"{calibration.fingerprint}.json"
    if path.is_file():
        existing = SO101Calibration.load(path)
        if existing.fingerprint != calibration.fingerprint:
            raise CalibrationError(
                f"calibration history collision at {path}; refusing to overwrite"
            )
        return path
    return calibration.save(path)


def save_versioned_calibration(
    calibration: SO101Calibration,
    *,
    robot_id: str,
    current_path: str | Path | None = None,
    history_dir: str | Path | None = None,
) -> tuple[Path, Path]:
    """Write the mutable current alias plus an immutable fingerprinted history copy."""

    current = calibration.save(current_path or default_calibration_path(robot_id))
    history = archive_calibration(
        calibration,
        robot_id=robot_id,
        history_dir=history_dir,
    )
    return current, history


def discover_lerobot_calibration(robot_id: str) -> Path | None:
    default_home = Path.home() / ".cache" / "huggingface" / "lerobot"
    base = Path(
        os.environ.get(
            "HF_LEROBOT_CALIBRATION",
            Path(os.environ.get("HF_LEROBOT_HOME", default_home)) / "calibration",
        )
    ).expanduser()
    candidates = (
        base / "robots" / "so101_follower" / f"{robot_id}.json",
        base / "robots" / "so100_follower" / f"{robot_id}.json",
        base / "so101_follower" / f"{robot_id}.json",
    )
    return next((candidate for candidate in candidates if candidate.is_file()), None)


def resolve_calibration(*, explicit_path: Path | None, robot_id: str) -> SO101Calibration | None:
    candidates: list[Path] = []
    if explicit_path is not None:
        candidates.append(explicit_path)
    candidates.append(default_calibration_path(robot_id))
    lerobot = discover_lerobot_calibration(robot_id)
    if lerobot is not None:
        candidates.append(lerobot)
    for candidate in candidates:
        if candidate.is_file():
            return SO101Calibration.load(candidate)
    return None
