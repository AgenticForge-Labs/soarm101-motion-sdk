"""One-time Feetech STS3215 motor ID and baud-rate setup."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from soarm101_motion.constants import EXPECTED_MODEL_NUMBER
from soarm101_motion.exceptions import CommunicationError, MissingDependencyError, RobotConnectionError

# Values are from the official FTServo Python SDK's ``sms_sts.py``.
STS3215_BAUDRATE_VALUES: dict[int, int] = {
    1_000_000: 0,
    500_000: 1,
    250_000: 2,
    128_000: 3,
    115_200: 4,
    76_800: 5,
    57_600: 6,
    38_400: 7,
}

_ID_ADDRESS = 5
_BAUD_RATE_ADDRESS = 6
_TORQUE_ENABLE_ADDRESS = 40
_LOCK_ADDRESS = 55
_MAX_MOTOR_ID = 253


@dataclass(frozen=True)
class MotorSetupResult:
    original_id: int
    original_baudrate: int
    target_id: int
    target_baudrate: int
    model_number: int


class FeetechMotorSetup:
    """Configure one physically isolated STS3215 motor.

    Only one motor may be attached during this operation. That constraint prevents
    duplicate factory IDs from being changed together.
    """

    def __init__(
        self,
        port: str,
        *,
        target_baudrate: int = 1_000_000,
        verify_model_number: bool = True,
    ) -> None:
        if target_baudrate not in STS3215_BAUDRATE_VALUES:
            supported = ", ".join(str(value) for value in STS3215_BAUDRATE_VALUES)
            raise ValueError(f"unsupported target baud rate {target_baudrate}; choose one of {supported}")
        self.port = port
        self.target_baudrate = int(target_baudrate)
        self.verify_model_number = verify_model_number

    @staticmethod
    def _load_sdk() -> tuple[Any, Any, int]:
        try:
            from scservo_sdk import COMM_SUCCESS, PortHandler, sms_sts
        except ImportError as exc:
            raise MissingDependencyError(
                "motor setup requires ftservo-python-sdk==2.0.0"
            ) from exc
        return PortHandler, sms_sts, int(COMM_SUCCESS)

    @staticmethod
    def _candidate_ids(initial_id: int | None) -> tuple[int, ...]:
        if initial_id is not None:
            if not 0 <= initial_id <= _MAX_MOTOR_ID:
                raise ValueError(f"initial motor ID must be in [0, {_MAX_MOTOR_ID}]")
            return (int(initial_id),)
        # Factory ID 1 is checked first, followed by the remaining legal IDs.
        return (1, 0, *range(2, _MAX_MOTOR_ID + 1))

    @staticmethod
    def _candidate_baudrates(initial_baudrate: int | None) -> tuple[int, ...]:
        if initial_baudrate is not None:
            if initial_baudrate not in STS3215_BAUDRATE_VALUES:
                supported = ", ".join(str(value) for value in STS3215_BAUDRATE_VALUES)
                raise ValueError(
                    f"unsupported initial baud rate {initial_baudrate}; choose one of {supported}"
                )
            return (int(initial_baudrate),)
        # Factory/common values first, then the rest of the official table.
        preferred = (1_000_000, 57_600, 115_200)
        return (*preferred, *(value for value in STS3215_BAUDRATE_VALUES if value not in preferred))

    @staticmethod
    def _check_result(packet: Any, comm_success: int, comm: int, error: int, operation: str) -> None:
        if comm != comm_success:
            raise CommunicationError(f"{operation}: {packet.getTxRxResult(comm)}")
        if error:
            raise CommunicationError(f"{operation}: {packet.getRxPacketError(error)}")

    def _find_single_motor(
        self,
        port_handler: Any,
        packet: Any,
        comm_success: int,
        *,
        initial_id: int | None,
        initial_baudrate: int | None,
    ) -> tuple[int, int, int]:
        found: list[tuple[int, int, int]] = []
        for baudrate in self._candidate_baudrates(initial_baudrate):
            if not port_handler.setBaudRate(baudrate):
                continue
            for motor_id in self._candidate_ids(initial_id):
                model, comm, error = packet.ping(motor_id)
                if comm == comm_success and not error:
                    found.append((baudrate, motor_id, int(model)))
                    if initial_id is not None and initial_baudrate is not None:
                        break
            if found and (initial_id is not None or initial_baudrate is not None):
                break

        unique = list(dict.fromkeys(found))
        if not unique:
            raise RobotConnectionError(
                "no motor found; connect exactly one powered STS3215 and provide its current "
                "--initial-id/--initial-baudrate if scanning is too slow"
            )
        if len(unique) > 1:
            detail = ", ".join(f"ID {mid} at {baud}" for baud, mid, _ in unique)
            raise RobotConnectionError(
                "multiple motor responses were detected; disconnect all but one motor: " + detail
            )
        return unique[0]

    def setup(
        self,
        *,
        target_id: int,
        initial_id: int | None = None,
        initial_baudrate: int | None = None,
    ) -> MotorSetupResult:
        if not 0 <= target_id <= _MAX_MOTOR_ID:
            raise ValueError(f"target motor ID must be in [0, {_MAX_MOTOR_ID}]")

        PortHandler, sms_sts, comm_success = self._load_sdk()
        port_handler = PortHandler(self.port)
        if not port_handler.openPort():
            raise RobotConnectionError(f"could not open serial port {self.port}")
        packet = sms_sts(port_handler)
        try:
            baudrate, motor_id, model = self._find_single_motor(
                port_handler,
                packet,
                comm_success,
                initial_id=initial_id,
                initial_baudrate=initial_baudrate,
            )
            if self.verify_model_number and model != EXPECTED_MODEL_NUMBER:
                raise RobotConnectionError(
                    f"expected STS3215 model {EXPECTED_MODEL_NUMBER}, found {model}"
                )
            if not port_handler.setBaudRate(baudrate):
                raise RobotConnectionError(f"could not set port baud rate to {baudrate}")

            comm, error = packet.write1ByteTxRx(motor_id, _TORQUE_ENABLE_ADDRESS, 0)
            self._check_result(packet, comm_success, comm, error, "disable torque")
            comm, error = packet.write1ByteTxRx(motor_id, _LOCK_ADDRESS, 0)
            self._check_result(packet, comm_success, comm, error, "unlock EEPROM")

            active_id = motor_id
            if active_id != target_id:
                comm, error = packet.write1ByteTxRx(active_id, _ID_ADDRESS, int(target_id))
                self._check_result(packet, comm_success, comm, error, "write motor ID")
                active_id = int(target_id)

            baud_value = STS3215_BAUDRATE_VALUES[self.target_baudrate]
            comm, error = packet.write1ByteTxRx(active_id, _BAUD_RATE_ADDRESS, baud_value)
            self._check_result(packet, comm_success, comm, error, "write motor baud rate")

            if not port_handler.setBaudRate(self.target_baudrate):
                raise RobotConnectionError(
                    f"could not switch host port to target baud rate {self.target_baudrate}"
                )
            verified_model, comm, error = packet.ping(active_id)
            self._check_result(packet, comm_success, comm, error, "verify configured motor")
            if self.verify_model_number and int(verified_model) != EXPECTED_MODEL_NUMBER:
                raise RobotConnectionError(
                    f"configured motor reported unexpected model {verified_model}"
                )
            comm, error = packet.write1ByteTxRx(active_id, _LOCK_ADDRESS, 1)
            self._check_result(packet, comm_success, comm, error, "lock EEPROM")
            return MotorSetupResult(
                original_id=motor_id,
                original_baudrate=baudrate,
                target_id=active_id,
                target_baudrate=self.target_baudrate,
                model_number=int(verified_model),
            )
        finally:
            port_handler.closePort()
