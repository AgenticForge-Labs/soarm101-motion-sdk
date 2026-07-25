"""Calibration storage and LeRobot-compatible conversion."""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass
from math import pi
from pathlib import Path
from typing import Any, Mapping

from soarm101_motion.constants import ALL_MOTORS, ENCODER_MAX, STOCK_GRIPPER
from soarm101_motion.exceptions import CalibrationError

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

    def raw_to_radians(self, raw: int) -> float:
        mid = (self.range_min + self.range_max) / 2.0
        degrees = (float(raw) - mid) * 360.0 / ENCODER_MAX
        return degrees * pi / 180.0

    def radians_to_raw(self, radians: float) -> int:
        mid = (self.range_min + self.range_max) / 2.0
        raw = int(round((float(radians) * 180.0 / pi) * ENCODER_MAX / 360.0 + mid))
        return min(self.range_max, max(self.range_min, raw))

    def raw_to_normalized(self, raw: int) -> float:
        bounded = min(self.range_max, max(self.range_min, int(raw)))
        value = (bounded - self.range_min) / (self.range_max - self.range_min)
        return 1.0 - value if self.drive_mode else value

    def normalized_to_raw(self, normalized: float) -> int:
        value = min(1.0, max(0.0, float(normalized)))
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

    @property
    def is_factory_range(self) -> bool:
        return all(
            calibration.range_min == 0 and calibration.range_max == ENCODER_MAX
            for calibration in self.motors.values()
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
