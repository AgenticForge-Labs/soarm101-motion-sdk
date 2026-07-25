"""SDK configuration."""

from dataclasses import dataclass
from pathlib import Path

from soarm101_motion.exceptions import ConfigurationError


@dataclass(frozen=True, slots=True)
class SOARM101Config:
    port: str | None = None
    robot_id: str = "soarm101"
    calibration_path: Path | None = None
    command_frequency_hz: float = 30.0
    connect_timeout_seconds: float = 10.0
    auto_enable_torque: bool = False
    strict_safety: bool = True

    def __post_init__(self) -> None:
        if not self.robot_id.strip():
            raise ConfigurationError("robot_id must not be empty")
        if self.command_frequency_hz <= 0:
            raise ConfigurationError("command_frequency_hz must be positive")
        if self.connect_timeout_seconds <= 0:
            raise ConfigurationError("connect_timeout_seconds must be positive")
