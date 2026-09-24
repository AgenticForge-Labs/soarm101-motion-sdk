from __future__ import annotations

import sys
import types
from collections import defaultdict

import pytest

from soarm101_motion.config import SOARM101Config
from soarm101_motion.constants import MOTOR_IDS
from soarm101_motion.exceptions import CalibrationError, CommunicationError, SafetyViolationError
from soarm101_motion.hardware.feetech import FeetechBackend


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
        for motor_id, raw in self.pending.items():
            self.packet.positions[motor_id] = raw
        return 0

    def clearParam(self) -> None:
        self.pending.clear()


class FakePacket:
    def __init__(self, port: FakePortHandler) -> None:
        self.port = port
        self.positions = {motor_id: 2047 for motor_id in MOTOR_IDS.values()}
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
        self.positions[motor_id] = raw
        return 0, 0

    def SyncWritePosEx(self, motor_id: int, raw: int, speed: int, acceleration: int):
        del speed, acceleration
        self.groupSyncWrite.pending[motor_id] = raw
        return True

    def unLockEprom(self, motor_id: int):
        del motor_id
        return 0, 0

    def LockEprom(self, motor_id: int):
        del motor_id
        return 0, 0

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


def test_connect_read_write_and_diagnostics(monkeypatch: pytest.MonkeyPatch) -> None:
    install_fake_sdk(monkeypatch)
    backend = FeetechBackend(
        SOARM101Config(
            port="FAKE",
            use_stored_calibration=False,
            verify_model_numbers=True,
        )
    )
    backend.connect()
    backend.enable_torque()
    before = backend.read_joint_positions()
    assert abs(before["shoulder_pan"]) < 0.01
    backend.write_joint_positions({"shoulder_pan": 0.25})
    assert backend.read_joint_positions()["shoulder_pan"] == pytest.approx(0.25, abs=0.002)
    backend.write_tool_position("so101_gripper", 0.75)
    assert backend.read_tool_position("so101_gripper") == pytest.approx(0.75, abs=0.002)
    diagnostics = backend.diagnostics()
    assert len(diagnostics) == 6
    assert all(item.model_number == 777 for item in diagnostics)
    backend.disconnect()
    assert not backend.is_connected


def test_voltage_snapshot_preserves_value_and_packet_fault(monkeypatch: pytest.MonkeyPatch) -> None:
    install_fake_sdk(monkeypatch)
    backend = FeetechBackend(
        SOARM101Config(port="FAKE", use_stored_calibration=False)
    )
    backend.connect()
    try:
        packet = backend._packet_handler
        motor_id = MOTOR_IDS["so101_gripper"]
        packet.registers[(motor_id, 62)] = 43
        packet.registers[(motor_id, 15)] = 40
        packet.registers[(motor_id, 14)] = 120
        original = packet.read1ByteTxRx

        def faulting_read(read_id: int, address: int):
            value, comm, error = original(read_id, address)
            if read_id == motor_id and address == 62:
                error = 1
            return value, comm, error

        monkeypatch.setattr(packet, "read1ByteTxRx", faulting_read)
        snapshot = backend.read_voltage_snapshot("so101_gripper", include_limits=True)
        assert snapshot["voltage_v"] == 4.3
        assert snapshot["minimum_voltage_v"] == 4.0
        assert snapshot["maximum_voltage_v"] == 12.0
        assert snapshot["readings"]["Present_Voltage"]["packet_error"] == 1
        with pytest.raises(CommunicationError, match="read Present_Voltage"):
            backend.read_register("so101_gripper", "Present_Voltage")
    finally:
        backend.disconnect()


def test_gripper_voltage_limit_write_is_scoped_and_relocked(monkeypatch: pytest.MonkeyPatch) -> None:
    install_fake_sdk(monkeypatch)
    backend = FeetechBackend(SOARM101Config(port="FAKE", use_stored_calibration=False))
    backend.connect()
    try:
        gripper_id = MOTOR_IDS["so101_gripper"]
        other_id = MOTOR_IDS["shoulder_pan"]
        backend._packet_handler.registers[(gripper_id, 15)] = 40
        backend._packet_handler.registers[(other_id, 15)] = 40
        with backend.eprom_unlocked("so101_gripper"):
            backend.write_register("so101_gripper", "Min_Voltage_Limit", 35)
        assert backend.read_register("so101_gripper", "Min_Voltage_Limit") == 35
        assert backend.read_register("shoulder_pan", "Min_Voltage_Limit") == 40
        assert backend.read_register("so101_gripper", "Lock") == 1
        assert backend.read_register("so101_gripper", "Torque_Enable") == 0
    finally:
        backend.disconnect()


def test_setup_connection_cannot_enable_uncalibrated_motor(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    install_fake_sdk(monkeypatch)

    class FactoryWristPacket(FakePacket):
        def __init__(self, port: FakePortHandler) -> None:
            super().__init__(port)
            wrist_id = MOTOR_IDS["wrist_roll"]
            self.registers[(wrist_id, 9)] = 0
            self.registers[(wrist_id, 11)] = 4095

    sys.modules["scservo_sdk"].sms_sts = FactoryWristPacket
    backend = FeetechBackend(
        SOARM101Config(
            port="FAKE",
            use_stored_calibration=False,
            allow_uncalibrated=True,
        )
    )
    backend.connect()
    packet = backend._packet_handler
    with pytest.raises(CalibrationError, match="wrist_roll"):
        backend.enable_torque()
    assert all(packet.registers[(motor_id, 40)] == 0 for motor_id in MOTOR_IDS.values())
    backend.disconnect()


def test_enable_refuses_present_position_outside_eeprom_limits(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    install_fake_sdk(monkeypatch)
    backend = FeetechBackend(
        SOARM101Config(port="FAKE", use_stored_calibration=False)
    )
    backend.connect()
    packet = backend._packet_handler
    packet.positions[MOTOR_IDS["shoulder_pan"]] = 50
    with pytest.raises(SafetyViolationError, match="shoulder_pan"):
        backend.enable_torque()
    assert all(packet.registers[(motor_id, 40)] == 0 for motor_id in MOTOR_IDS.values())
    backend.disconnect()
