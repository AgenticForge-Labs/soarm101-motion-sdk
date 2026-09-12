"""Public Feetech backend with safe torque/EEPROM separation."""

from __future__ import annotations

import logging
import time
from collections.abc import Sequence

from soarm101_motion.calibration import SO101Calibration
from soarm101_motion.calibration_live import EncoderSweep, calibration_from_sweeps
from soarm101_motion.constants import ALL_MOTORS
from soarm101_motion.exceptions import CalibrationError, CommunicationError
from soarm101_motion.hardware.feetech import FeetechBackend as _ProtocolFeetechBackend

logger = logging.getLogger(__name__)


class FeetechBackend(_ProtocolFeetechBackend):
    """SO-ARM101 backend that never unlocks EEPROM during ordinary torque changes.

    Register ``Lock=0`` is reserved for the inherited ``eprom_unlocked`` context
    used by explicit setup, configuration, and calibration operations. Relaxing,
    disconnecting, or rolling back a failed torque enable keeps EEPROM locked.
    """

    def enable_torque(self, motors: Sequence[str] | None = None) -> None:
        with self._io_lock:
            self._require_connected()
            selected = tuple(motors) if motors is not None else ALL_MOTORS
            unknown = set(selected) - set(ALL_MOTORS)
            if unknown:
                raise KeyError(next(iter(unknown)))

            # Latch measured positions before energizing any servo.
            raw_positions = {name: self.read_raw_position(name) for name in selected}
            self._write_raw_positions(raw_positions, speed_raw=1, acceleration_raw=1)

            enabled: list[str] = []
            try:
                for name in selected:
                    # Lock EEPROM before torque is enabled. Normal operation never
                    # needs EEPROM writes and should leave the persistent area protected.
                    self.write_register(name, "Lock", 1)
                    self.write_register(name, "Torque_Enable", 1)
                    enabled.append(name)
            except BaseException:
                for name in reversed(enabled):
                    try:
                        self.write_register(name, "Torque_Enable", 0)
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
            unknown = set(selected) - set(ALL_MOTORS)
            if unknown:
                raise KeyError(next(iter(unknown)))

            errors: list[str] = []
            for name in selected:
                try:
                    self.write_register(name, "Torque_Enable", 0)
                except Exception as exc:
                    errors.append(f"{name} torque: {exc}")

            if motors is None or set(selected) == set(ALL_MOTORS):
                self._torque_enabled = False
            if errors:
                raise CommunicationError(
                    "failed to disable all selected motors: " + "; ".join(errors)
                )

    def interactive_calibration(
        self,
        *,
        record_seconds: float = 20.0,
        poll_interval: float = 0.02,
    ) -> SO101Calibration:
        """Calibrate all motors from one live sweep between printed end stops.

        Torque is disabled and the old homing/range values are temporarily reset.
        The user then moves every motor repeatedly through its complete safe range.
        Encoder positions are unwrapped while sampling, so crossing the 4095/0 seam
        is supported.  Only after both extrema are known is the midpoint calculated
        and written as the motor homing reference.

        EEPROM changes are transactional: any failure attempts to restore the exact
        calibration snapshot that existed before the sweep.
        """

        if record_seconds <= 0:
            raise CalibrationError("calibration sweep duration must be positive")
        if poll_interval <= 0:
            raise CalibrationError("calibration poll interval must be positive")

        with self._io_lock:
            self._require_connected()
            previous_calibration = self.calibration
            eeprom_snapshot = self.read_calibration_from_motors()
            self.disable_torque()
            try:
                # Observe the physical mechanism in raw encoder space.  Do not ask
                # the user to guess the midpoint before the true stops are known.
                self.reset_calibration()
                start = self.read_all_raw_positions()
                sweeps = {name: EncoderSweep.start(raw) for name, raw in start.items()}

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
