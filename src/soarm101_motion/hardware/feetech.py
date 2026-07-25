"""Direct Feetech STS3215 backend using the official Python SDK package."""

from __future__ import annotations

import logging
import time
from collections.abc import Mapping, Sequence
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterator

from soarm101_motion.calibration import (
    MotorCalibration,
    SO101Calibration,
    default_calibration_path,
    resolve_calibration,
)
from soarm101_motion.config import SOARM101Config
from soarm101_motion.constants import (
    ALL_MOTORS,
    ARM_JOINTS,
    ENCODER_MAX,
    EXPECTED_MODEL_NUMBER,
    HALF_TURN,
    MOTOR_IDS,
    STOCK_GRIPPER,
    STS3215_REGISTERS,
)
from soarm101_motion.exceptions import (
    CalibrationError,
    CommunicationError,
    MissingDependencyError,
    RobotConnectionError,
)
from soarm101_motion.hardware.base import SO101HardwareBackend
from soarm101_motion.types import HardwareState, MotorDiagnostic

logger = logging.getLogger(__name__)


class FeetechBackend(SO101HardwareBackend):
    """SO-ARM101 follower backend built directly on ``ftservo-python-sdk``."""

    realtime = True

    def __init__(self, config: SOARM101Config) -> None:
        if not config.port:
            raise RobotConnectionError("a serial port is required for FeetechBackend")
        self.config = config
        self.calibration = (
            resolve_calibration(explicit_path=config.calibration_path, robot_id=config.robot_id)
            if config.use_stored_calibration
            else None
        )
        self._connected = False
        self._torque_enabled = False
        self._port_handler: Any = None
        self._packet_handler: Any = None
        self._comm_success: int = 0

    @property
    def is_connected(self) -> bool:
        return self._connected

    @staticmethod
    def candidate_ports() -> list[str]:
        try:
            from serial.tools import list_ports
        except ImportError as exc:
            raise MissingDependencyError("serial port discovery requires pyserial") from exc
        ports = list(list_ports.comports())
        likely = [
            item.device
            for item in ports
            if any(token in (item.description or "").lower() for token in ("ch340", "usb", "serial"))
        ]
        return likely or [item.device for item in ports]

    def _load_sdk(self) -> tuple[Any, Any, int]:
        try:
            from scservo_sdk import COMM_SUCCESS, PortHandler, sms_sts
        except ImportError as exc:
            raise MissingDependencyError(
                "Feetech hardware support requires 'ftservo-python-sdk>=2.0'"
            ) from exc
        return PortHandler, sms_sts, int(COMM_SUCCESS)

    def connect(self) -> None:
        if self._connected:
            return
        PortHandler, sms_sts, self._comm_success = self._load_sdk()
        self._port_handler = PortHandler(self.config.port)
        self._packet_handler = sms_sts(self._port_handler)
        if not self._port_handler.openPort():
            raise RobotConnectionError(f"could not open serial port {self.config.port}")
        if not self._port_handler.setBaudRate(self.config.baudrate):
            self._port_handler.closePort()
            raise RobotConnectionError(f"could not set baud rate {self.config.baudrate}")
        try:
            self._verify_motors()
            motor_calibration = self.read_calibration_from_motors()
            if self.calibration is None:
                self.calibration = motor_calibration
            else:
                self.calibration.validate()
                if self.config.verify_calibration_on_connect:
                    self._verify_calibration_matches_motors(self.calibration, motor_calibration)
            if self.calibration.is_factory_range and not self.config.allow_uncalibrated:
                raise CalibrationError(
                    "motors appear uncalibrated (all ranges are 0..4095); run "
                    "'soarm101 calibrate --port PORT'"
                )
            if self.config.configure_motors_on_connect:
                self.configure_motors()
            self._connected = True
        except Exception:
            self._port_handler.closePort()
            self._port_handler = None
            self._packet_handler = None
            raise

    def disconnect(self) -> None:
        if self._port_handler is None:
            return
        try:
            if self._connected and self.config.disable_torque_on_disconnect:
                self.disable_torque()
        finally:
            self._port_handler.closePort()
            self._connected = False
            self._torque_enabled = False
            self._port_handler = None
            self._packet_handler = None

    def _verify_motors(self) -> None:
        missing: list[str] = []
        wrong_model: list[str] = []
        for name, motor_id in MOTOR_IDS.items():
            model, comm, error = self._packet_handler.ping(motor_id)
            if comm != self._comm_success:
                missing.append(f"{name} (ID {motor_id})")
                continue
            if error:
                raise CommunicationError(
                    f"motor {name} ping returned status error: "
                    f"{self._packet_handler.getRxPacketError(error)}"
                )
            if self.config.verify_model_numbers and int(model) != EXPECTED_MODEL_NUMBER:
                wrong_model.append(f"{name}: expected {EXPECTED_MODEL_NUMBER}, found {model}")
        if missing or wrong_model:
            details: list[str] = []
            if missing:
                details.append("missing motors: " + ", ".join(missing))
            if wrong_model:
                details.append("wrong model numbers: " + "; ".join(wrong_model))
            raise RobotConnectionError("; ".join(details))

    def _check_result(self, comm: int, error: int, operation: str) -> None:
        if comm != self._comm_success:
            raise CommunicationError(f"{operation}: {self._packet_handler.getTxRxResult(comm)}")
        if error:
            raise CommunicationError(f"{operation}: {self._packet_handler.getRxPacketError(error)}")

    def read_register(self, motor: str, register: str) -> int:
        if motor not in MOTOR_IDS:
            raise KeyError(motor)
        if register not in STS3215_REGISTERS:
            raise KeyError(register)
        address, width = STS3215_REGISTERS[register]
        motor_id = MOTOR_IDS[motor]
        if width == 1:
            value, comm, error = self._packet_handler.read1ByteTxRx(motor_id, address)
        elif width == 2:
            value, comm, error = self._packet_handler.read2ByteTxRx(motor_id, address)
        else:
            raise NotImplementedError(f"unsupported register width {width}")
        self._check_result(comm, error, f"read {register} from {motor}")
        if register == "Homing_Offset":
            value = self._packet_handler.scs_tohost(value, 11)
        elif register in {"Present_Position", "Present_Velocity", "Goal_Position", "Goal_Velocity"}:
            value = self._packet_handler.scs_tohost(value, 15)
        return int(value)

    def write_register(self, motor: str, register: str, value: int) -> None:
        if motor not in MOTOR_IDS:
            raise KeyError(motor)
        if register not in STS3215_REGISTERS:
            raise KeyError(register)
        address, width = STS3215_REGISTERS[register]
        motor_id = MOTOR_IDS[motor]
        encoded = int(value)
        if register == "Homing_Offset":
            encoded = self._packet_handler.scs_toscs(encoded, 11)
        elif register in {"Goal_Position", "Goal_Velocity"}:
            encoded = self._packet_handler.scs_toscs(encoded, 15)
        if width == 1:
            comm, error = self._packet_handler.write1ByteTxRx(motor_id, address, encoded)
        elif width == 2:
            comm, error = self._packet_handler.write2ByteTxRx(motor_id, address, encoded)
        else:
            raise NotImplementedError(f"unsupported register width {width}")
        self._check_result(comm, error, f"write {register} on {motor}")

    @contextmanager
    def eprom_unlocked(self, motor: str) -> Iterator[None]:
        self.write_register(motor, "Torque_Enable", 0)
        self.write_register(motor, "Lock", 0)
        try:
            yield
        finally:
            self.write_register(motor, "Lock", 1)

    def read_calibration_from_motors(self) -> SO101Calibration:
        motors: dict[str, MotorCalibration] = {}
        for name, motor_id in MOTOR_IDS.items():
            motors[name] = MotorCalibration(
                motor_id=motor_id,
                drive_mode=0,
                homing_offset=self.read_register(name, "Homing_Offset"),
                range_min=self.read_register(name, "Min_Position_Limit"),
                range_max=self.read_register(name, "Max_Position_Limit"),
            )
        return SO101Calibration(motors=motors, source="motor-eeprom")

    @staticmethod
    def _verify_calibration_matches_motors(
        file_calibration: SO101Calibration,
        motor_calibration: SO101Calibration,
    ) -> None:
        mismatches: list[str] = []
        for name in ALL_MOTORS:
            expected = file_calibration.motors[name]
            actual = motor_calibration.motors[name]
            if (
                expected.homing_offset != actual.homing_offset
                or expected.range_min != actual.range_min
                or expected.range_max != actual.range_max
            ):
                mismatches.append(name)
        if mismatches:
            raise CalibrationError(
                "stored calibration does not match motor EEPROM for: " + ", ".join(mismatches)
            )

    def apply_calibration(self, calibration: SO101Calibration) -> None:
        calibration.validate()
        was_enabled = self._torque_enabled
        if was_enabled:
            self.disable_torque()
        for name, value in calibration.motors.items():
            with self.eprom_unlocked(name):
                self.write_register(name, "Homing_Offset", value.homing_offset)
                self.write_register(name, "Min_Position_Limit", value.range_min)
                self.write_register(name, "Max_Position_Limit", value.range_max)
        self.calibration = calibration
        if was_enabled:
            self.enable_torque()

    def reset_calibration(self) -> None:
        motors: dict[str, MotorCalibration] = {}
        for name, motor_id in MOTOR_IDS.items():
            with self.eprom_unlocked(name):
                self.write_register(name, "Homing_Offset", 0)
                self.write_register(name, "Min_Position_Limit", 0)
                self.write_register(name, "Max_Position_Limit", ENCODER_MAX)
            motors[name] = MotorCalibration(motor_id, 0, 0, 0, ENCODER_MAX)
        self.calibration = SO101Calibration(motors=motors, source="factory-range")

    def configure_motors(self) -> None:
        for name in ALL_MOTORS:
            with self.eprom_unlocked(name):
                self.write_register(name, "Operating_Mode", 0)
                self.write_register(name, "Return_Delay_Time", 0)
                self.write_register(name, "Maximum_Acceleration", 254)
                self.write_register(name, "P_Coefficient", self.config.position_p_coefficient)
                self.write_register(name, "I_Coefficient", self.config.position_i_coefficient)
                self.write_register(name, "D_Coefficient", self.config.position_d_coefficient)
                phase = self.read_register(name, "Phase")
                if phase & 0x10:
                    self.write_register(name, "Phase", phase & ~0x10)
        with self.eprom_unlocked(STOCK_GRIPPER):
            self.write_register(STOCK_GRIPPER, "Max_Torque_Limit", 500)
            self.write_register(STOCK_GRIPPER, "Protection_Current", 250)
            self.write_register(STOCK_GRIPPER, "Overload_Torque", 25)

    def _require_calibration(self) -> SO101Calibration:
        if self.calibration is None:
            raise CalibrationError("no calibration is loaded")
        return self.calibration

    def read_raw_position(self, motor: str) -> int:
        raw, comm, error = self._packet_handler.ReadPos(MOTOR_IDS[motor])
        self._check_result(comm, error, f"read position from {motor}")
        return int(raw)

    def read_all_raw_positions(self) -> dict[str, int]:
        return {name: self.read_raw_position(name) for name in ALL_MOTORS}

    def read_joint_positions(self) -> dict[str, float]:
        calibration = self._require_calibration()
        return {
            name: calibration.motors[name].raw_to_radians(self.read_raw_position(name))
            for name in ARM_JOINTS
        }

    def write_joint_positions(
        self,
        positions: Mapping[str, float],
        *,
        speed_raw: int | None = None,
        acceleration_raw: int | None = None,
    ) -> None:
        calibration = self._require_calibration()
        speed = speed_raw if speed_raw is not None else self.config.hardware_speed_raw
        acceleration = (
            acceleration_raw
            if acceleration_raw is not None
            else self.config.hardware_acceleration_raw
        )
        if not positions:
            return
        for name, position in positions.items():
            if name not in ARM_JOINTS:
                raise KeyError(name)
            raw = calibration.motors[name].radians_to_raw(position)
            if not self._packet_handler.SyncWritePosEx(MOTOR_IDS[name], raw, speed, acceleration):
                self._packet_handler.groupSyncWrite.clearParam()
                raise CommunicationError(f"could not add {name} to synchronous write")
        comm = self._packet_handler.groupSyncWrite.txPacket()
        self._packet_handler.groupSyncWrite.clearParam()
        if comm != self._comm_success:
            raise CommunicationError(self._packet_handler.getTxRxResult(comm))

    def read_tool_position(self, actuator: str) -> float:
        calibration = self._require_calibration()
        return calibration.motors[actuator].raw_to_normalized(self.read_raw_position(actuator))

    def write_tool_position(
        self,
        actuator: str,
        position: float,
        *,
        speed_raw: int | None = None,
        acceleration_raw: int | None = None,
    ) -> None:
        calibration = self._require_calibration()
        raw = calibration.motors[actuator].normalized_to_raw(position)
        comm, error = self._packet_handler.WritePosEx(
            MOTOR_IDS[actuator],
            raw,
            speed_raw if speed_raw is not None else self.config.hardware_speed_raw,
            acceleration_raw
            if acceleration_raw is not None
            else self.config.hardware_acceleration_raw,
        )
        self._check_result(comm, error, f"move tool actuator {actuator}")

    def enable_torque(self, motors: Sequence[str] | None = None) -> None:
        selected = tuple(motors) if motors is not None else ALL_MOTORS
        for name in selected:
            self.write_register(name, "Lock", 1)
            self.write_register(name, "Torque_Enable", 1)
        if motors is None or set(selected) == set(ALL_MOTORS):
            self._torque_enabled = True

    def disable_torque(self, motors: Sequence[str] | None = None) -> None:
        selected = tuple(motors) if motors is not None else ALL_MOTORS
        for name in selected:
            self.write_register(name, "Torque_Enable", 0)
            self.write_register(name, "Lock", 0)
        if motors is None or set(selected) == set(ALL_MOTORS):
            self._torque_enabled = False

    def stop(self) -> None:
        if not self._torque_enabled or not self._connected:
            return
        self.write_joint_positions(self.read_joint_positions(), speed_raw=1, acceleration_raw=1)

    def get_hardware_state(self) -> HardwareState:
        moving = False
        faults: list[str] = []
        if self._connected:
            for name in ALL_MOTORS:
                try:
                    moving = moving or bool(self.read_register(name, "Moving"))
                    status = self.read_register(name, "Status")
                    if status:
                        faults.append(f"{name}: status 0x{status:02x}")
                except CommunicationError as exc:
                    faults.append(str(exc))
        return HardwareState(
            connected=self._connected,
            torque_enabled=self._torque_enabled,
            moving=moving,
            faulted=bool(faults),
            fault_message="; ".join(faults) or None,
        )

    def diagnostics(self) -> list[MotorDiagnostic]:
        diagnostics: list[MotorDiagnostic] = []
        for name, motor_id in MOTOR_IDS.items():
            try:
                diagnostics.append(
                    MotorDiagnostic(
                        name=name,
                        motor_id=motor_id,
                        model_number=self.read_register(name, "Model_Number"),
                        position_raw=self.read_raw_position(name),
                        temperature_c=float(self.read_register(name, "Present_Temperature")),
                        voltage_v=self.read_register(name, "Present_Voltage") / 10.0,
                        current_raw=self.read_register(name, "Present_Current"),
                        moving=bool(self.read_register(name, "Moving")),
                        status=self.read_register(name, "Status"),
                    )
                )
            except Exception as exc:
                diagnostics.append(
                    MotorDiagnostic(
                        name=name,
                        motor_id=motor_id,
                        model_number=None,
                        position_raw=None,
                        temperature_c=None,
                        voltage_v=None,
                        current_raw=None,
                        moving=None,
                        status=None,
                        error=str(exc),
                    )
                )
        return diagnostics

    def interactive_calibration(
        self,
        *,
        record_seconds: float = 20.0,
        poll_interval: float = 0.02,
    ) -> SO101Calibration:
        """Calibrate after the operator has manually placed the arm at its center.

        This method performs no prompts. The CLI handles operator confirmation and
        calls this method immediately after the arm is centered.
        """
        self.disable_torque()
        self.reset_calibration()
        center_before_offset = self.read_all_raw_positions()
        offsets = {name: raw - HALF_TURN for name, raw in center_before_offset.items()}
        for name, offset in offsets.items():
            with self.eprom_unlocked(name):
                self.write_register(name, "Homing_Offset", offset)
        start = self.read_all_raw_positions()
        mins = dict(start)
        maxes = dict(start)
        deadline = time.monotonic() + record_seconds
        while time.monotonic() < deadline:
            values = self.read_all_raw_positions()
            mins = {name: min(mins[name], values[name]) for name in ALL_MOTORS}
            maxes = {name: max(maxes[name], values[name]) for name in ALL_MOTORS}
            time.sleep(poll_interval)
        mins["wrist_roll"] = 0
        maxes["wrist_roll"] = ENCODER_MAX
        unmoved = [name for name in ALL_MOTORS if name != "wrist_roll" and mins[name] == maxes[name]]
        if unmoved:
            raise CalibrationError("no range was recorded for: " + ", ".join(unmoved))
        motors = {
            name: MotorCalibration(
                motor_id=MOTOR_IDS[name],
                drive_mode=0,
                homing_offset=offsets[name],
                range_min=mins[name],
                range_max=maxes[name],
            )
            for name in ALL_MOTORS
        }
        calibration = SO101Calibration(motors=motors, source="interactive-calibration")
        self.apply_calibration(calibration)
        return calibration

    def save_calibration(
        self,
        calibration: SO101Calibration,
        *,
        path: Path | None = None,
        lerobot_path: Path | None = None,
    ) -> Path:
        saved = calibration.save(path or default_calibration_path(self.config.robot_id))
        if lerobot_path is not None:
            calibration.save_lerobot(lerobot_path)
        return saved
