from __future__ import annotations

from dataclasses import replace

import pytest

from soarm101_motion import SOARM101, SOARM101Config
from soarm101_motion.constants import HOME_JOINTS
from soarm101_motion.exceptions import SafetyViolationError
from soarm101_motion.hardware.setup import FeetechMotorSetup
from soarm101_motion.kinematics.model import SO101KinematicModel
from soarm101_motion.safety import validate_workspace_configuration


def test_normal_connections_do_not_configure_motors() -> None:
    assert SOARM101Config().configure_motors_on_connect is False


def test_software_stop_name_is_honest() -> None:
    arm = SOARM101.simulated()
    assert callable(arm.software_stop)
    assert not hasattr(arm, "emergency_stop")


def test_workspace_home_passes_and_foldback_is_rejected() -> None:
    model = SO101KinematicModel()
    validate_workspace_configuration(model, HOME_JOINTS)
    foldback = {
        "shoulder_pan": 0.4763863841,
        "shoulder_lift": 1.0743894884,
        "elbow_flex": 1.5364325048,
        "wrist_flex": 1.649,
        "wrist_roll": -0.8608,
    }
    with pytest.raises(SafetyViolationError, match="self-clearance"):
        validate_workspace_configuration(model, foldback)


def test_workspace_checks_can_be_disabled_for_model_development() -> None:
    config = replace(SOARM101Config(), enable_workspace_checks=False)
    assert config.enable_workspace_checks is False


class FakePort:
    def __init__(self, _: str) -> None:
        self.baudrate = 1_000_000
        self.opened = False

    def openPort(self) -> bool:
        self.opened = True
        return True

    def closePort(self) -> None:
        self.opened = False

    def setBaudRate(self, baudrate: int) -> bool:
        self.baudrate = int(baudrate)
        return True


class FakePacket:
    BAUDS = {
        0: 1_000_000,
        1: 500_000,
        2: 250_000,
        3: 128_000,
        4: 115_200,
        5: 76_800,
        6: 57_600,
        7: 38_400,
    }

    def __init__(self, port: FakePort) -> None:
        self.port = port
        self.motor_id = 1
        self.motor_baudrate = 57_600
        self.model = 777
        self.locked = True
        self.torque_enabled = False

    def ping(self, motor_id: int):
        if self.port.baudrate == self.motor_baudrate and motor_id == self.motor_id:
            return self.model, 0, 0
        return 0, -1, 0

    def write1ByteTxRx(self, motor_id: int, address: int, value: int):
        if self.port.baudrate != self.motor_baudrate or motor_id != self.motor_id:
            return -1, 0
        if address == 40:
            self.torque_enabled = bool(value)
        elif address == 55:
            self.locked = bool(value)
        elif address == 5:
            self.motor_id = int(value)
        elif address == 6:
            self.motor_baudrate = self.BAUDS[int(value)]
        return 0, 0

    @staticmethod
    def getTxRxResult(comm: int) -> str:
        return f"comm={comm}"

    @staticmethod
    def getRxPacketError(error: int) -> str:
        return f"error={error}"


def test_one_motor_setup_changes_id_and_baud(monkeypatch: pytest.MonkeyPatch) -> None:
    created: list[FakePacket] = []

    def packet_factory(port: FakePort) -> FakePacket:
        packet = FakePacket(port)
        created.append(packet)
        return packet

    monkeypatch.setattr(
        FeetechMotorSetup,
        "_load_sdk",
        staticmethod(lambda: (FakePort, packet_factory, 0)),
    )
    result = FeetechMotorSetup("FAKE").setup(
        target_id=6,
        initial_id=1,
        initial_baudrate=57_600,
    )
    assert result.original_id == 1
    assert result.original_baudrate == 57_600
    assert result.target_id == 6
    assert result.target_baudrate == 1_000_000
    assert created[0].motor_id == 6
    assert created[0].motor_baudrate == 1_000_000
    assert created[0].locked is True
