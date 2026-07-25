"""Deterministic kinematic simulation backend."""

from __future__ import annotations

from collections.abc import Mapping, Sequence

from soarm101_motion.calibration import SO101Calibration
from soarm101_motion.constants import ALL_MOTORS, HOME_JOINTS, MOTOR_IDS, STOCK_GRIPPER
from soarm101_motion.exceptions import InvalidCommandError, RobotConnectionError
from soarm101_motion.hardware.base import SO101HardwareBackend
from soarm101_motion.types import HardwareState, MotorDiagnostic


class SimulationBackend(SO101HardwareBackend):
    """In-memory backend using the same planner, FK, IK, tools, and safety as hardware."""

    realtime = False

    def __init__(
        self,
        initial_positions: Mapping[str, float] | None = None,
        *,
        realtime: bool = False,
    ) -> None:
        self.calibration: SO101Calibration | None = None
        self.realtime = realtime
        self._connected = False
        self._torque_enabled = False
        self._moving = False
        self._positions = dict(HOME_JOINTS)
        if initial_positions:
            self._positions.update({name: float(value) for name, value in initial_positions.items()})
        self._tool_positions = {STOCK_GRIPPER: 1.0}
        self.command_history: list[dict[str, float]] = []

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False
        self._torque_enabled = False
        self._moving = False

    def _require_connected(self) -> None:
        if not self._connected:
            raise RobotConnectionError("simulation backend is not connected")

    def read_joint_positions(self) -> dict[str, float]:
        self._require_connected()
        return self._positions.copy()

    def write_joint_positions(
        self,
        positions: Mapping[str, float],
        *,
        speed_raw: int | None = None,
        acceleration_raw: int | None = None,
    ) -> None:
        del speed_raw, acceleration_raw
        self._require_connected()
        if not self._torque_enabled:
            raise InvalidCommandError("torque is disabled")
        self._moving = True
        self._positions.update({name: float(value) for name, value in positions.items()})
        self.command_history.append(self._positions.copy())
        self._moving = False

    def read_tool_position(self, actuator: str) -> float:
        self._require_connected()
        return self._tool_positions[actuator]

    def write_tool_position(
        self,
        actuator: str,
        position: float,
        *,
        speed_raw: int | None = None,
        acceleration_raw: int | None = None,
    ) -> None:
        del speed_raw, acceleration_raw
        self._require_connected()
        if not self._torque_enabled:
            raise InvalidCommandError("torque is disabled")
        self._tool_positions[actuator] = min(1.0, max(0.0, float(position)))

    def enable_torque(self, motors: Sequence[str] | None = None) -> None:
        del motors
        self._require_connected()
        self._torque_enabled = True

    def disable_torque(self, motors: Sequence[str] | None = None) -> None:
        del motors
        self._require_connected()
        self._torque_enabled = False
        self._moving = False

    def stop(self) -> None:
        self._moving = False

    def get_hardware_state(self) -> HardwareState:
        return HardwareState(
            connected=self._connected,
            torque_enabled=self._torque_enabled,
            moving=self._moving,
            faulted=False,
        )

    def diagnostics(self) -> list[MotorDiagnostic]:
        return [
            MotorDiagnostic(
                name=name,
                motor_id=MOTOR_IDS[name],
                model_number=777,
                position_raw=None,
                temperature_c=25.0,
                voltage_v=7.4,
                current_raw=0,
                moving=self._moving,
                status=0,
            )
            for name in ALL_MOTORS
        ]
