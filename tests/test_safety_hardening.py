from __future__ import annotations

import sys
import threading
import time
import types
from collections import defaultdict

import numpy as np
import pytest

from soarm101_motion import SOARM101, SOARM101Config
from soarm101_motion.calibration import MotorCalibration, SO101Calibration
from soarm101_motion.constants import ARM_JOINTS, MOTOR_IDS
from soarm101_motion.exceptions import (
    CalibrationError,
    InvalidCommandError,
    MotionCancelledError,
    MotionTimeoutError,
)
from soarm101_motion.hardware.feetech import FeetechBackend
from soarm101_motion.hardware.simulation import SimulationBackend
from soarm101_motion.kinematics.ik import _axis_angle_residual


def test_antiparallel_axis_error_is_pi() -> None:
    residual = _axis_angle_residual(
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, 0.0, -1.0]),
    )
    assert np.linalg.norm(residual) == pytest.approx(np.pi, abs=1e-8)


def test_blocking_motion_can_be_stopped() -> None:
    arm = SOARM101.simulated(realtime=True)
    caught: list[BaseException] = []
    with arm:
        arm.enable()

        def run() -> None:
            try:
                arm.move_joints([0.5, -0.8, 0.8, 0.5, 0.3], speed=0.05)
            except BaseException as exc:
                caught.append(exc)

        thread = threading.Thread(target=run)
        thread.start()
        deadline = time.monotonic() + 1.0
        while not arm.is_moving and time.monotonic() < deadline:
            time.sleep(0.005)
        assert arm.is_moving
        arm.stop()
        thread.join(timeout=1.0)
        assert not thread.is_alive()
        assert len(caught) == 1
        assert isinstance(caught[0], MotionCancelledError)
        count = len(arm.backend.command_history)  # type: ignore[attr-defined]
        time.sleep(0.05)
        assert len(arm.backend.command_history) == count  # type: ignore[attr-defined]


def test_joint_trajectory_is_speed_and_acceleration_limited() -> None:
    arm = SOARM101.simulated(realtime=False)
    with arm:
        arm.enable()
        arm.move_joints(
            [0.5, -0.4, 0.3, 0.2, -0.1],
            speed=0.2,
            acceleration=0.4,
        )
        history = [
            {name: 0.0 for name in ARM_JOINTS},
            *arm.backend.command_history,  # type: ignore[attr-defined]
        ]
    matrix = np.array([[sample[name] for name in ARM_JOINTS] for sample in history])
    dt = 1.0 / arm.config.command_frequency_hz
    velocity = np.diff(matrix, axis=0) / dt
    acceleration = np.diff(velocity, axis=0) / dt
    assert np.max(np.abs(velocity)) <= 0.201
    assert np.max(np.abs(acceleration)) <= 0.401


class StalledSimulationBackend(SimulationBackend):
    realtime = True

    def write_joint_positions(self, positions, **kwargs):
        del positions, kwargs
        self._require_connected()
        if not self._torque_enabled:
            raise InvalidCommandError("torque is disabled")


def test_wait_true_requires_feedback_completion() -> None:
    backend = StalledSimulationBackend(realtime=True)
    config = SOARM101Config(
        motion_completion_timeout_s=0.08,
        settle_time_s=0.01,
        feedback_poll_interval_s=0.005,
    )
    arm = SOARM101(config, backend=backend)
    with arm:
        arm.enable()
        with pytest.raises(MotionTimeoutError):
            arm.move_joints([0.1, 0, 0, 0, 0], speed=1.0, acceleration=5.0)


class FakePortHandler:
    def __init__(self, port: str) -> None:
        self.port = port
        self.open = False
        self.baudrate = None

    def openPort(self) -> bool:
        self.open = True
        return True

    def closePort(self) -> None:
        self.open = False

    def setBaudRate(self, baudrate: int) -> bool:
        self.baudrate = baudrate
        return True


class FakeGroupSyncWrite:
    def __init__(self, packet: "FakePacket") -> None:
        self.packet = packet
        self.pending: dict[int, int] = {}

    def txPacket(self) -> int:
        self.packet.goals.update(self.pending)
        return 0

    def clearParam(self) -> None:
        self.pending.clear()


class FakePacket:
    def __init__(self, port: FakePortHandler) -> None:
        self.port = port
        self.positions = {motor_id: 2047 for motor_id in MOTOR_IDS.values()}
        self.goals = {motor_id: 123 for motor_id in MOTOR_IDS.values()}
        self.registers = defaultdict(int)
        for motor_id in MOTOR_IDS.values():
            self.registers[(motor_id, 3)] = 777
            self.registers[(motor_id, 9)] = 100
            self.registers[(motor_id, 11)] = 3995
            self.registers[(motor_id, 62)] = 74
            self.registers[(motor_id, 63)] = 25
        self.groupSyncWrite = FakeGroupSyncWrite(self)

    @staticmethod
    def scs_tohost(value: int, bit: int) -> int:
        return -(value & ~(1 << bit)) if value & (1 << bit) else value

    @staticmethod
    def scs_toscs(value: int, bit: int) -> int:
        return -value | (1 << bit) if value < 0 else value

    def ping(self, motor_id: int):
        return 777, 0, 0

    def read1ByteTxRx(self, motor_id: int, address: int):
        return self.registers[(motor_id, address)], 0, 0

    def read2ByteTxRx(self, motor_id: int, address: int):
        if address == 56:
            return self.positions[motor_id], 0, 0
        return self.registers[(motor_id, address)], 0, 0

    def write1ByteTxRx(self, motor_id: int, address: int, value: int):
        self.registers[(motor_id, address)] = value
        return 0, 0

    def write2ByteTxRx(self, motor_id: int, address: int, value: int):
        self.registers[(motor_id, address)] = value
        return 0, 0

    def ReadPos(self, motor_id: int):
        return self.positions[motor_id], 0, 0

    def WritePosEx(self, motor_id: int, raw: int, speed: int, acceleration: int):
        del speed, acceleration
        self.goals[motor_id] = raw
        return 0, 0

    def SyncWritePosEx(self, motor_id: int, raw: int, speed: int, acceleration: int):
        del speed, acceleration
        self.groupSyncWrite.pending[motor_id] = raw
        return True

    @staticmethod
    def getTxRxResult(comm: int) -> str:
        return f"comm={comm}"

    @staticmethod
    def getRxPacketError(error: int) -> str:
        return f"error={error}"


def install_fake_sdk(monkeypatch: pytest.MonkeyPatch) -> None:
    module = types.ModuleType("scservo_sdk")
    module.COMM_SUCCESS = 0
    module.PortHandler = FakePortHandler
    module.sms_sts = FakePacket
    monkeypatch.setitem(sys.modules, "scservo_sdk", module)


def make_backend(monkeypatch: pytest.MonkeyPatch) -> FeetechBackend:
    install_fake_sdk(monkeypatch)
    backend = FeetechBackend(
        SOARM101Config(
            port="FAKE",
            use_stored_calibration=False,
            verify_model_numbers=True,
            configure_motors_on_connect=False,
        )
    )
    backend.connect()
    return backend


def test_hardware_rejects_torque_off_write_and_safe_enable_latches_goals(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    backend = make_backend(monkeypatch)
    with pytest.raises(InvalidCommandError):
        backend.write_joint_positions({"shoulder_pan": 0.1})
    packet = backend._packet_handler
    assert set(packet.goals.values()) == {123}
    backend.enable_torque()
    assert packet.goals == packet.positions
    backend.disconnect()


def test_calibration_failure_restores_eeprom(monkeypatch: pytest.MonkeyPatch) -> None:
    backend = make_backend(monkeypatch)
    before = backend.read_calibration_from_motors()
    with pytest.raises(CalibrationError):
        backend.interactive_calibration(record_seconds=0.0)
    after = backend.read_calibration_from_motors()
    for name in MOTOR_IDS:
        assert after.motors[name].range_min == before.motors[name].range_min
        assert after.motors[name].range_max == before.motors[name].range_max
        assert after.motors[name].homing_offset == before.motors[name].homing_offset
    backend.disconnect()


def test_factory_range_detection_is_per_motor() -> None:
    motors = {
        name: MotorCalibration(motor_id, 0, 1, 100, 3995)
        for name, motor_id in MOTOR_IDS.items()
    }
    motors["elbow_flex"] = MotorCalibration(MOTOR_IDS["elbow_flex"], 0, 0, 0, 4095)
    calibration = SO101Calibration(motors)
    assert calibration.uncalibrated_motors == ("elbow_flex",)
