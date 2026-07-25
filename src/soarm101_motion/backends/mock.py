"""Deterministic mock backend for local development and tests."""

from collections.abc import Mapping

from soarm101_motion.exceptions import RobotConnectionError
from soarm101_motion.types import HardwareState

JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


class MockSOARM101Backend:
    def __init__(self) -> None:
        self._connected = False
        self._torque_enabled = False
        self._positions = {name: 0.0 for name in JOINT_NAMES}
        self.command_history: list[dict[str, float]] = []
        self.fail_next_command = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False
        self._torque_enabled = False

    def read_joint_positions(self) -> Mapping[str, float]:
        self._require_connection()
        return dict(self._positions)

    def read_hardware_state(self) -> HardwareState:
        return HardwareState(
            connected=self._connected,
            torque_enabled=self._torque_enabled,
            moving=False,
            faulted=False,
        )

    def write_joint_positions(self, positions: Mapping[str, float]) -> None:
        self._require_connection()
        if self.fail_next_command:
            self.fail_next_command = False
            raise RuntimeError("simulated communication failure")
        command = {name: float(value) for name, value in positions.items()}
        self._positions.update(command)
        self.command_history.append(command)

    def enable_torque(self) -> None:
        self._require_connection()
        self._torque_enabled = True

    def disable_torque(self) -> None:
        self._require_connection()
        self._torque_enabled = False

    def stop(self) -> None:
        self._require_connection()

    def _require_connection(self) -> None:
        if not self._connected:
            raise RobotConnectionError("backend is not connected")
