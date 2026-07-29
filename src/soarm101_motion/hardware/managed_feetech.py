"""Public Feetech backend with safe torque/EEPROM separation."""

from __future__ import annotations

import logging
from collections.abc import Sequence

from soarm101_motion.constants import ALL_MOTORS
from soarm101_motion.exceptions import CommunicationError
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
