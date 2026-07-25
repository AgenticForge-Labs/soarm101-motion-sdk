"""Primary application-facing robot facade."""

import time
from collections.abc import Mapping

from soarm101_motion.backends.base import RobotBackend
from soarm101_motion.config import SOARM101Config
from soarm101_motion.exceptions import CommunicationError, HardwareFaultError, RobotConnectionError
from soarm101_motion.safety import DEFAULT_JOINT_LIMITS, validate_joint_targets
from soarm101_motion.types import HardwareState, JointState, MotionResult


class SOARM101:
    def __init__(
        self,
        config: SOARM101Config | None = None,
        *,
        backend: RobotBackend,
    ) -> None:
        self.config = config or SOARM101Config()
        self._backend = backend

    @property
    def is_connected(self) -> bool:
        return self._backend.is_connected

    def connect(self) -> None:
        if self.is_connected:
            return
        try:
            self._backend.connect()
            if self.config.auto_enable_torque:
                self._backend.enable_torque()
        except Exception as exc:
            raise RobotConnectionError("failed to connect to SO-ARM101") from exc

    def disconnect(self) -> None:
        if self.is_connected:
            self._backend.disconnect()

    def get_joint_positions(self) -> JointState:
        self._require_connection()
        positions = dict(self._backend.read_joint_positions())
        return JointState(positions=positions, timestamp=time.monotonic())

    def get_hardware_state(self) -> HardwareState:
        return self._backend.read_hardware_state()

    def move_joints(self, positions: Mapping[str, float]) -> MotionResult:
        self._require_connection()
        state = self.get_hardware_state()
        if state.faulted:
            raise HardwareFaultError(state.fault_message or "robot is faulted")
        validated = validate_joint_targets(positions, DEFAULT_JOINT_LIMITS)
        try:
            self._backend.write_joint_positions(validated)
        except Exception as exc:
            raise CommunicationError("failed to send joint targets") from exc
        return MotionResult(accepted=True, completed=True)

    def stop(self) -> None:
        self._require_connection()
        self._backend.stop()

    def hold(self) -> None:
        self._require_connection()
        positions = self.get_joint_positions().positions
        self._backend.enable_torque()
        self._backend.write_joint_positions(positions)

    def relax(self) -> None:
        self._require_connection()
        self._backend.disable_torque()

    def run_diagnostics(self) -> HardwareState:
        return self.get_hardware_state()

    def _require_connection(self) -> None:
        if not self.is_connected:
            raise RobotConnectionError("SO-ARM101 is not connected")
