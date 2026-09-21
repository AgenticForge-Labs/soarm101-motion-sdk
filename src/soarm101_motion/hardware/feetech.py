"""Direct Feetech STS3215 backend using the official Python SDK package."""

from __future__ import annotations

import logging
import threading
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
from soarm101_motion.calibration_live import EncoderSweep, calibration_from_sweeps
from soarm101_motion.config import SOARM101Config
from soarm101_motion.constants import (
    ALL_MOTORS,
    ARM_JOINTS,
    ENCODER_MAX,
    EXPECTED_MODEL_NUMBER,
    MOTOR_IDS,
    STOCK_GRIPPER,
    STS3215_REGISTERS,
)
from soarm101_motion.exceptions import (
    CalibrationError,
    CommunicationError,
    InvalidCommandError,
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
        self._io_lock = threading.RLock()

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

    def _require_transport(self) -> None:
        if self._port_handler is None or self._packet_handler is None:
            raise RobotConnectionError("Feetech serial transport is not open")

    def _require_connected(self) -> None:
        if not self._connected:
            raise RobotConnectionError("SO-ARM101 is not connected")

    def connect(self) -> None:
        with self._io_lock:
            if self._connected:
                return
            PortHandler, sms_sts, self._comm_success = self._load_sdk()
            self._port_handler = PortHandler(self.config.port)
            self._packet_handler = sms_sts(self._port_handler)
            if not self._port_handler.openPort():
                raise RobotConnectionError(f"could not open serial port {self.config.port}")
            if not self._port_handler.setBaudRate(self.config.baudrate):
                self._port_handler.closePort()
                self._port_handler = None
                self._packet_handler = None
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
                uncalibrated = self.calibration.uncalibrated_motors
                if uncalibrated and not self.config.allow_uncalibrated:
                    raise CalibrationError(
                        "motors appear uncalibrated: "
                        + ", ".join(uncalibrated)
                        + "; run 'soarm101 calibrate --port PORT'"
                    )
                if self.config.configure_motors_on_connect:
                    self.configure_motors()
                self._connected = True
            except Exception:
                self._connected = False
                self._port_handler.closePort()
                self._port_handler = None
                self._packet_handler = None
                raise

    def disconnect(self) -> None:
        pending_error: BaseException | None = None
        with self._io_lock:
            if self._port_handler is None:
                return
            try:
                if self._connected and self.config.disable_torque_on_disconnect:
                    try:
                        self.disable_torque()
                    except BaseException as exc:
                        pending_error = exc
            finally:
                try:
                    self._port_handler.closePort()
                finally:
                    self._connected = False
                    self._torque_enabled = False
                    self._port_handler = None
                    self._packet_handler = None
        if pending_error is not None:
            raise pending_error

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
        with self._io_lock:
            self._require_transport()
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
        with self._io_lock:
            self._require_transport()
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
        with self._io_lock:
            self.write_register(motor, "Torque_Enable", 0)
            self.write_register(motor, "Lock", 0)
            try:
                yield
            finally:
                self.write_register(motor, "Lock", 1)

    def read_calibration_from_motors(self) -> SO101Calibration:
        with self._io_lock:
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
        with self._io_lock:
            calibration.validate()
            was_enabled = self._torque_enabled
            if was_enabled:
                self.disable_torque()
            try:
                for name, value in calibration.motors.items():
                    with self.eprom_unlocked(name):
                        self.write_register(name, "Homing_Offset", value.homing_offset)
                        self.write_register(name, "Min_Position_Limit", value.range_min)
                        self.write_register(name, "Max_Position_Limit", value.range_max)
                self.calibration = calibration
            finally:
                if was_enabled:
                    self.enable_torque()

    def reset_calibration(self) -> None:
        with self._io_lock:
            motors: dict[str, MotorCalibration] = {}
            for name, motor_id in MOTOR_IDS.items():
                with self.eprom_unlocked(name):
                    self.write_register(name, "Homing_Offset", 0)
                    self.write_register(name, "Min_Position_Limit", 0)
                    self.write_register(name, "Max_Position_Limit", ENCODER_MAX)
                motors[name] = MotorCalibration(motor_id, 0, 0, 0, ENCODER_MAX)
            self.calibration = SO101Calibration(motors=motors, source="factory-range")

    def configure_motors(self) -> None:
        with self._io_lock:
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
        with self._io_lock:
            self._require_transport()
            raw, comm, error = self._packet_handler.ReadPos(MOTOR_IDS[motor])
            self._check_result(comm, error, f"read position from {motor}")
            return int(raw)

    def read_all_raw_positions(self) -> dict[str, int]:
        with self._io_lock:
            return {name: self.read_raw_position(name) for name in ALL_MOTORS}

    def read_joint_positions(self) -> dict[str, float]:
        with self._io_lock:
            self._require_connected()
            calibration = self._require_calibration()
            return {
                name: calibration.motors[name].raw_to_radians(self.read_raw_position(name))
                for name in ARM_JOINTS
            }

    def _write_raw_positions(
        self,
        positions: Mapping[str, int],
        *,
        speed_raw: int,
        acceleration_raw: int,
    ) -> None:
        with self._io_lock:
            self._require_transport()
            if not positions:
                return
            self._packet_handler.groupSyncWrite.clearParam()
            try:
                for name, raw in positions.items():
                    if name not in MOTOR_IDS:
                        raise KeyError(name)
                    if not self._packet_handler.SyncWritePosEx(
                        MOTOR_IDS[name], int(raw), int(speed_raw), int(acceleration_raw)
                    ):
                        raise CommunicationError(f"could not add {name} to synchronous write")
                comm = self._packet_handler.groupSyncWrite.txPacket()
                if comm != self._comm_success:
                    raise CommunicationError(self._packet_handler.getTxRxResult(comm))
            finally:
                self._packet_handler.groupSyncWrite.clearParam()

    def write_joint_positions(
        self,
        positions: Mapping[str, float],
        *,
        speed_raw: int | None = None,
        acceleration_raw: int | None = None,
    ) -> None:
        with self._io_lock:
            self._require_connected()
            if not self._torque_enabled:
                raise InvalidCommandError("torque is disabled; call enable_torque() before motion")
            calibration = self._require_calibration()
            raw_positions: dict[str, int] = {}
            for name, position in positions.items():
                if name not in ARM_JOINTS:
                    raise KeyError(name)
                raw_positions[name] = calibration.motors[name].radians_to_raw(position)
            self._write_raw_positions(
                raw_positions,
                speed_raw=speed_raw if speed_raw is not None else self.config.hardware_speed_raw,
                acceleration_raw=(
                    acceleration_raw
                    if acceleration_raw is not None
                    else self.config.hardware_acceleration_raw
                ),
            )

    def read_tool_position(self, actuator: str) -> float:
        with self._io_lock:
            self._require_connected()
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
        with self._io_lock:
            self._require_connected()
            if not self._torque_enabled:
                raise InvalidCommandError("torque is disabled; call enable_torque() before tool motion")
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
        with self._io_lock:
            self._require_connected()
            selected = tuple(motors) if motors is not None else ALL_MOTORS
            unknown = set(selected) - set(ALL_MOTORS)
            if unknown:
                raise KeyError(next(iter(unknown)))
            raw_positions = {name: self.read_raw_position(name) for name in selected}
            self._write_raw_positions(raw_positions, speed_raw=1, acceleration_raw=1)
            enabled: list[str] = []
            try:
                for name in selected:
                    self.write_register(name, "Torque_Enable", 1)
                    enabled.append(name)
                    self.write_register(name, "Lock", 1)
            except BaseException:
                for name in reversed(enabled):
                    try:
                        self.write_register(name, "Torque_Enable", 0)
                        self.write_register(name, "Lock", 0)
                    except Exception:
                        logger.exception("failed to roll back torque enable for %s", name)
                self._torque_enabled = False
                raise
            if motors is None or set(selected) == set(ALL_MOTORS):
                self._torque_enabled = True

    def disable_torque(self, motors: Sequence[str] | None = None) -> None:
        with self._io_lock:
            self._require_connected()
            selected = tuple(motors) if motors is not None else ALL_MOTORS
            errors: list[str] = []
            for name in selected:
                try:
                    self.write_register(name, "Torque_Enable", 0)
                except Exception as exc:
                    errors.append(f"{name} torque: {exc}")
                try:
                    self.write_register(name, "Lock", 0)
                except Exception as exc:
                    errors.append(f"{name} lock: {exc}")
            if motors is None or set(selected) == set(ALL_MOTORS):
                self._torque_enabled = False
            if errors:
                raise CommunicationError("failed to disable all selected motors: " + "; ".join(errors))

    def stop(self) -> None:
        with self._io_lock:
            if not self._torque_enabled or not self._connected:
                return
            raw_positions = {name: self.read_raw_position(name) for name in ALL_MOTORS}
            self._write_raw_positions(raw_positions, speed_raw=1, acceleration_raw=1)

    def get_hardware_state(self) -> HardwareState:
        with self._io_lock:
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
        with self._io_lock:
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
        """Calibrate from repeated mechanical-stop sweeps.

        Torque remains disabled. Raw encoders are sampled with zero homing offsets
        and unrestricted position limits, and each encoder is unwrapped across the
        4095/0 seam. The physical zero is the midpoint between the observed extrema.
        EEPROM changes are transactional and restored on failure when possible.
        """
        if record_seconds <= 0:
            raise ValueError("record_seconds must be positive")
        if poll_interval <= 0:
            raise ValueError("poll_interval must be positive")
        with self._io_lock:
            self._require_connected()
            previous_calibration = self.calibration
            eeprom_snapshot = self.read_calibration_from_motors()
            self.disable_torque()
            try:
                self.reset_calibration()
                initial = self.read_all_raw_positions()
                sweeps = {
                    name: EncoderSweep.start(raw)
                    for name, raw in initial.items()
                }
                deadline = time.monotonic() + record_seconds
                while time.monotonic() < deadline:
                    values = self.read_all_raw_positions()
                    for name, raw in values.items():
                        sweeps[name].update(raw)
                    time.sleep(poll_interval)

                calibration = calibration_from_sweeps(sweeps)
                self.apply_calibration(calibration)
                verified = self.read_calibration_from_motors()
                self._verify_calibration_matches_motors(calibration, verified)
                return calibration
            except BaseException:
                try:
                    self.apply_calibration(eeprom_snapshot)
                    self.calibration = previous_calibration or eeprom_snapshot
                except Exception as rollback_exc:
                    raise CalibrationError(
                        "calibration failed and EEPROM rollback also failed; do not enable torque"
                    ) from rollback_exc
                raise

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
