from __future__ import annotations

import sys
import types
from collections import defaultdict

import pytest

from soarm101_motion.config import SOARM101Config
from soarm101_motion.constants import MOTOR_IDS
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
