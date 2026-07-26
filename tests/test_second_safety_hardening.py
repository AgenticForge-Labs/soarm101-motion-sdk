from __future__ import annotations

import threading
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from soarm101_motion import Pose, SOARM101, SOARM101Config
from soarm101_motion.exceptions import (
    CommunicationError,
    InvalidCommandError,
    MotionCancelledError,
    RobotConnectionError,
    SafetyViolationError,
)
from soarm101_motion.hardware.simulation import SimulationBackend
from soarm101_motion.kinematics import IKOptions, IKSolver
from test_safety_hardening import make_backend


class FailingWriteBackend(SimulationBackend):
    def __init__(self) -> None:
        super().__init__(realtime=False)
        self.write_count = 0

    def write_joint_positions(self, positions, **kwargs):
        self.write_count += 1
        if self.write_count == 3:
            raise CommunicationError("injected serial failure")
        super().write_joint_positions(positions, **kwargs)


def test_motion_failure_best_effort_stops_backend() -> None:
    backend = FailingWriteBackend()
    arm = SOARM101(SOARM101Config(), backend=backend)
    with arm:
        arm.enable()
        with pytest.raises(CommunicationError):
            arm.move_joints([0.2, 0, 0, 0, 0], speed=0.2, acceleration=0.5)
        assert backend.stop_count >= 1


class LaggingBackend(SimulationBackend):
    def write_joint_positions(self, positions, **kwargs):
        del positions, kwargs
        self._require_connected()
        if not self._torque_enabled:
            raise InvalidCommandError("torque is disabled")


def test_following_error_aborts_during_streaming() -> None:
    backend = LaggingBackend(realtime=False)
    config = SOARM101Config(
        following_error_limit_rad=0.01,
        trajectory_feedback_interval_s=0.02,
    )
    arm = SOARM101(config, backend=backend)
    with arm:
        arm.enable()
        with pytest.raises(SafetyViolationError, match="following error"):
            arm.move_joints([0.2, 0, 0, 0, 0], speed=0.2, acceleration=0.5)
        assert backend.stop_count >= 1


def test_command_overrides_cannot_exceed_safety_ceiling() -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        with pytest.raises(SafetyViolationError, match="safety maximum"):
            arm.move_joints(
                [0.1, 0, 0, 0, 0],
                speed=arm.config.max_joint_speed + 0.1,
            )


def test_compatible_orientation_keeps_axes_independent() -> None:
    solver = IKSolver()
    actual = Pose.identity()
    target = Pose(np.zeros(3), Rotation.from_euler("xyz", [1.2, -0.7, 0.8]).as_matrix())
    residual = solver._orientation_residual(
        actual,
        target,
        IKOptions(orientation_mode="compatible"),
    )
    assert residual.shape == (6,)
    assert np.linalg.norm(residual[:3]) > 0.1
    assert np.linalg.norm(residual[3:]) > 0.01


class StalledToolBackend(SimulationBackend):
    realtime = True

    def write_tool_position(self, actuator, position, **kwargs):
        del actuator, position, kwargs
        self._require_connected()
        if not self._torque_enabled:
            raise InvalidCommandError("torque is disabled")


def test_arm_stop_cancels_active_gripper_motion() -> None:
    backend = StalledToolBackend(realtime=True)
    arm = SOARM101(SOARM101Config(), backend=backend)
    with arm:
        arm.enable()
        handle = arm.tool.close(wait=False)
        deadline = time.monotonic() + 1.0
        while not arm.is_moving and time.monotonic() < deadline:
            time.sleep(0.005)
        arm.stop()
        with pytest.raises(MotionCancelledError):
            handle.wait(1.0)
        assert backend.stop_count >= 1


def test_simulation_rejects_invalid_tool_target() -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        with pytest.raises(InvalidCommandError):
            arm.tool.move(1.2)


class EnableFailureBackend(SimulationBackend):
    def __init__(self) -> None:
        super().__init__()
        self.disconnected_after_failure = False

    def enable_torque(self, motors=None) -> None:
        del motors
        raise CommunicationError("injected enable failure")

    def disconnect(self) -> None:
        self.disconnected_after_failure = True
        super().disconnect()


def test_auto_enable_failure_disconnects_backend() -> None:
    backend = EnableFailureBackend()
    arm = SOARM101(
        SOARM101Config(auto_enable_torque=True),
        backend=backend,
    )
    with pytest.raises(RobotConnectionError):
        arm.connect()
    assert backend.disconnected_after_failure
    assert not backend.is_connected


def test_feetech_enable_rolls_back_partial_success(monkeypatch: pytest.MonkeyPatch) -> None:
    backend = make_backend(monkeypatch)
    packet = backend._packet_handler
    original = packet.write1ByteTxRx

    def fail_third_motor(motor_id: int, address: int, value: int):
        if address == 40 and motor_id == 3 and value == 1:
            return 1, 0
        return original(motor_id, address, value)

    packet.write1ByteTxRx = fail_third_motor
    with pytest.raises(CommunicationError):
        backend.enable_torque()
    assert packet.registers[(1, 40)] == 0
    assert packet.registers[(2, 40)] == 0
    assert not backend.get_hardware_state().torque_enabled
    backend.disconnect()


def test_feetech_sync_writes_are_serialized(monkeypatch: pytest.MonkeyPatch) -> None:
    backend = make_backend(monkeypatch)
    backend.enable_torque()
    packet = backend._packet_handler
    original = packet.SyncWritePosEx
    concurrent = 0
    maximum = 0
    guard = threading.Lock()

    def delayed(*args):
        nonlocal concurrent, maximum
        with guard:
            concurrent += 1
            maximum = max(maximum, concurrent)
        time.sleep(0.002)
        try:
            return original(*args)
        finally:
            with guard:
                concurrent -= 1

    packet.SyncWritePosEx = delayed
    errors: list[BaseException] = []

    def write(value: float) -> None:
        try:
            backend.write_joint_positions({"shoulder_pan": value})
        except BaseException as exc:
            errors.append(exc)

    first = threading.Thread(target=write, args=(0.1,))
    second = threading.Thread(target=write, args=(-0.1,))
    first.start()
    second.start()
    first.join()
    second.join()
    assert not errors
    assert maximum == 1
    backend.disconnect()
