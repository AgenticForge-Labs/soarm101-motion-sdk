"""Backend contract shared by hardware and simulators."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Mapping, Sequence

from soarm101_motion.calibration import SO101Calibration
from soarm101_motion.types import HardwareState, MotorDiagnostic


class SO101HardwareBackend(ABC):
    calibration: SO101Calibration | None

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @property
    @abstractmethod
    def is_connected(self) -> bool: ...

    @abstractmethod
    def read_joint_positions(self) -> dict[str, float]: ...

    @abstractmethod
    def write_joint_positions(
        self,
        positions: Mapping[str, float],
        *,
        speed_raw: int | None = None,
        acceleration_raw: int | None = None,
    ) -> None: ...

    @abstractmethod
    def read_tool_position(self, actuator: str) -> float: ...

    @abstractmethod
    def write_tool_position(
        self,
        actuator: str,
        position: float,
        *,
        speed_raw: int | None = None,
        acceleration_raw: int | None = None,
    ) -> None: ...

    @abstractmethod
    def enable_torque(self, motors: Sequence[str] | None = None) -> None: ...

    @abstractmethod
    def disable_torque(self, motors: Sequence[str] | None = None) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...

    @abstractmethod
    def get_hardware_state(self) -> HardwareState: ...

    @abstractmethod
    def diagnostics(self) -> list[MotorDiagnostic]: ...
