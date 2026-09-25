"""Public Feetech backend with safe torque/EEPROM separation."""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable, Mapping, Sequence

from soarm101_motion.calibration import SO101Calibration
from soarm101_motion.calibration_live import (
    EncoderSweep,
    calibration_from_sweeps,
    display_travel_targets,
    sweep_progress_snapshot,
)
from soarm101_motion.config import SOARM101Config
from soarm101_motion.constants import ALL_MOTORS
from soarm101_motion.exceptions import CalibrationCancelledError, CalibrationError, CommunicationError, SafetyViolationError
from soarm101_motion.hardware.feetech import FeetechBackend as _ProtocolFeetechBackend
from soarm101_motion.types import HardwareState

logger = logging.getLogger(__name__)


class FeetechBackend(_ProtocolFeetechBackend):
    """SO-ARM101 backend with guarded EEPROM and motor-effort safety interlocks.

    Register ``Lock=0`` is reserved for the inherited ``eprom_unlocked`` context
    used by explicit setup, configuration, and calibration operations. Relaxing,
    disconnecting, or rolling back a failed torque enable keeps EEPROM locked.

    While torque is enabled, the backend also monitors STS3215 current and load
    feedback. Repeated threshold violations latch a software safety interlock,
    hold the measured positions, and make the normal motion controller abort.
    This is contact/collision detection from motor effort, not calibrated force.
    """

    def __init__(self, config: SOARM101Config) -> None:
        super().__init__(config)
        self._effort_trip_message: str | None = None
        self._effort_violation_counts = dict.fromkeys(ALL_MOTORS, 0)
        self._effort_safety_enabled = bool(config.effort_safety_enabled)
        self._effort_current_trip_raw = config.effort_current_trip_raw
        self._effort_load_trip_raw = config.effort_load_trip_raw
        self._effort_trip_consecutive_samples = config.effort_trip_consecutive_samples
        self._motor_current_trip_raw = dict(config.motor_current_trip_raw)
        self._motor_load_trip_raw = dict(config.motor_load_trip_raw)
        self._last_effort_readings: dict[str, dict[str, int]] = {}
        self._effort_peaks = {
            name: {"current_raw": 0, "abs_load_raw": 0} for name in ALL_MOTORS
        }
        self._effort_sample_monotonic: float | None = None
        self._gripper_contact_latched = False

    def _ensure_effort_state(self) -> None:
        """Initialize safety-latch state for normal and lightweight backend instances."""

        # Some transport-only tests deliberately construct this class with
        # object.__new__ so they can exercise torque ordering without opening a
        # serial port. Lazy initialization keeps those instances compatible and
        # also makes older deserialized/backend wrappers safe to use.
        if not hasattr(self, "_effort_trip_message"):
            self._effort_trip_message = None
        if not hasattr(self, "_effort_violation_counts"):
            self._effort_violation_counts = dict.fromkeys(ALL_MOTORS, 0)
        config = getattr(self, "config", None)
        if not hasattr(self, "_effort_safety_enabled"):
            self._effort_safety_enabled = bool(
                getattr(config, "effort_safety_enabled", True)
            )
        if not hasattr(self, "_effort_current_trip_raw"):
            self._effort_current_trip_raw = getattr(
                config, "effort_current_trip_raw", 250
            )
        if not hasattr(self, "_effort_load_trip_raw"):
            self._effort_load_trip_raw = getattr(
                config, "effort_load_trip_raw", 850
            )
        if not hasattr(self, "_effort_trip_consecutive_samples"):
            self._effort_trip_consecutive_samples = int(
                getattr(config, "effort_trip_consecutive_samples", 2)
            )
        if not hasattr(self, "_motor_current_trip_raw"):
            self._motor_current_trip_raw = dict(
                getattr(config, "motor_current_trip_raw", {})
            )
        if not hasattr(self, "_motor_load_trip_raw"):
            self._motor_load_trip_raw = dict(
                getattr(config, "motor_load_trip_raw", {})
            )
        if not hasattr(self, "_last_effort_readings"):
            self._last_effort_readings = {}
        if not hasattr(self, "_effort_peaks"):
            self._effort_peaks = {
                name: {"current_raw": 0, "abs_load_raw": 0} for name in ALL_MOTORS
            }
        if not hasattr(self, "_effort_sample_monotonic"):
            self._effort_sample_monotonic = None
        if not hasattr(self, "_gripper_contact_latched"):
            self._gripper_contact_latched = False

    def set_gripper_contact_latched(self, latched: bool) -> None:
        """Treat gripper effort as expected while teleop holds object contact.

        Hardware status faults remain active. This only prevents the software
        effort threshold for the gripper from aborting the other arm joints.
        """
        with self._io_lock:
            self._ensure_effort_state()
            self._gripper_contact_latched = bool(latched)
            self._effort_violation_counts["so101_gripper"] = 0

    def _require_effort_clear(self) -> None:
        self._ensure_effort_state()
        if self._effort_trip_message is not None:
            raise SafetyViolationError(
                self._effort_trip_message
                + "; remove the obstruction and call clear_effort_trip() before motion"
            )

    @staticmethod
    def _decode_present_load(raw: int) -> int:
        """Decode STS3215 Present_Load sign-magnitude encoding (sign bit 10)."""

        value = int(raw)
        magnitude = value & 0x03FF
        return -magnitude if value & 0x0400 else magnitude

    def _current_limit(self, motor: str) -> int | None:
        self._ensure_effort_state()
        return self._motor_current_trip_raw.get(
            motor,
            self._effort_current_trip_raw,
        )

    def _load_limit(self, motor: str) -> int | None:
        self._ensure_effort_state()
        return self._motor_load_trip_raw.get(
            motor,
            self._effort_load_trip_raw,
        )

    def read_motor_effort(self, motor: str) -> dict[str, int]:
        """Return raw STS3215 current and signed load feedback for one motor."""

        with self._io_lock:
            self._require_connected()
            if motor not in ALL_MOTORS:
                raise KeyError(motor)
            current = abs(self.read_register(motor, "Present_Current"))
            load = self._decode_present_load(self.read_register(motor, "Present_Load"))
            return {"current_raw": current, "load_raw": load}

    def _read_all_motor_effort(
        self, *, skip_motors: set[str] | None = None
    ) -> dict[str, dict[str, int]]:
        skipped = skip_motors or set()
        readings = {
            name: self.read_motor_effort(name)
            for name in ALL_MOTORS
            if name not in skipped
        }
        self._last_effort_readings.update(
            {name: dict(values) for name, values in readings.items()}
        )
        self._effort_sample_monotonic = time.monotonic()
        for name, values in readings.items():
            peak = self._effort_peaks[name]
            peak["current_raw"] = max(peak["current_raw"], abs(int(values["current_raw"])))
            peak["abs_load_raw"] = max(peak["abs_load_raw"], abs(int(values["load_raw"])))
        return readings

    def get_effort_safety_status(self, *, refresh: bool = False) -> dict[str, object]:
        """Return live/cached effort guard settings, readings, peaks, and trip state."""

        with self._io_lock:
            self._ensure_effort_state()
            if refresh:
                if not self._connected:
                    raise CommunicationError("robot is not connected")
                self._read_all_motor_effort()
            return {
                "supported": True,
                "enabled": self._effort_safety_enabled,
                "current_trip_raw": self._effort_current_trip_raw,
                "load_trip_raw": self._effort_load_trip_raw,
                "consecutive_samples": self._effort_trip_consecutive_samples,
                "trip_message": self._effort_trip_message,
                "sample_monotonic": self._effort_sample_monotonic,
                "readings": {
                    name: dict(values)
                    for name, values in self._last_effort_readings.items()
                },
                "peaks": {
                    name: dict(values) for name, values in self._effort_peaks.items()
                },
                "effective_limits": {
                    name: {
                        "current_raw": self._current_limit(name),
                        "load_raw": self._load_limit(name),
                    }
                    for name in ALL_MOTORS
                },
            }

    def configure_effort_safety(
        self,
        *,
        enabled: bool,
        current_trip_raw: int | None,
        load_trip_raw: int | None,
        consecutive_samples: int,
    ) -> None:
        """Apply session-only global effort thresholds while torque is disabled."""

        with self._io_lock:
            self._ensure_effort_state()
            if self._torque_enabled:
                raise SafetyViolationError(
                    "disable torque before changing effort-safety settings"
                )
            if current_trip_raw is not None and int(current_trip_raw) <= 0:
                raise ValueError("current trip threshold must be positive or disabled")
            if load_trip_raw is not None and not 1 <= int(load_trip_raw) <= 1023:
                raise ValueError("load trip threshold must be in [1, 1023] or disabled")
            if int(consecutive_samples) < 1:
                raise ValueError("consecutive effort samples must be at least 1")
            self._effort_safety_enabled = bool(enabled)
            self._effort_current_trip_raw = (
                None if current_trip_raw is None else int(current_trip_raw)
            )
            self._effort_load_trip_raw = (
                None if load_trip_raw is None else int(load_trip_raw)
            )
            self._effort_trip_consecutive_samples = int(consecutive_samples)
            self._effort_violation_counts = dict.fromkeys(ALL_MOTORS, 0)

    def reset_effort_peaks(self) -> None:
        with self._io_lock:
            self._ensure_effort_state()
            self._effort_peaks = {
                name: {"current_raw": 0, "abs_load_raw": 0} for name in ALL_MOTORS
            }

    def _sample_effort_trip(self) -> str | None:
        self._ensure_effort_state()
        if not self._effort_safety_enabled:
            return None

        required = self._effort_trip_consecutive_samples
        skipped = {"so101_gripper"} if self._gripper_contact_latched else set()
        readings = self._read_all_motor_effort(skip_motors=skipped)
        tripped: list[str] = []
        for name in ALL_MOTORS:
            if name in skipped:
                self._effort_violation_counts[name] = 0
                continue
            current_limit = self._current_limit(name)
            load_limit = self._load_limit(name)
            if current_limit is None and load_limit is None:
                self._effort_violation_counts[name] = 0
                continue

            reading = readings[name]
            current = reading["current_raw"]
            load = reading["load_raw"]
            violations: list[str] = []
            if current_limit is not None and current >= current_limit:
                violations.append(f"current {current} >= {current_limit}")
            if load_limit is not None and abs(load) >= load_limit:
                violations.append(f"|load| {abs(load)} >= {load_limit}")

            if violations:
                self._effort_violation_counts[name] += 1
                if self._effort_violation_counts[name] >= required:
                    tripped.append(f"{name}: " + ", ".join(violations))
            else:
                self._effort_violation_counts[name] = 0

        if not tripped:
            return None
        return "motor effort safety trip: " + "; ".join(tripped)

    @property
    def effort_trip_message(self) -> str | None:
        self._ensure_effort_state()
        return self._effort_trip_message

    def clear_effort_trip(self) -> None:
        """Clear the latched software effort trip after the obstruction is removed."""

        with self._io_lock:
            self._ensure_effort_state()
            self._effort_trip_message = None
            self._effort_violation_counts = dict.fromkeys(ALL_MOTORS, 0)

    def write_joint_positions(
        self,
        positions: Mapping[str, float],
        *,
        speed_raw: int | None = None,
        acceleration_raw: int | None = None,
    ) -> None:
        """Reject direct joint writes while an effort safety trip is latched."""

        with self._io_lock:
            self._require_effort_clear()
            super().write_joint_positions(
                positions,
                speed_raw=speed_raw,
                acceleration_raw=acceleration_raw,
            )

    def write_tool_position(
        self,
        actuator: str,
        position: float,
        *,
        speed_raw: int | None = None,
        acceleration_raw: int | None = None,
    ) -> None:
        """Reject direct tool writes while an effort safety trip is latched."""

        with self._io_lock:
            self._require_effort_clear()
            super().write_tool_position(
                actuator,
                position,
                speed_raw=speed_raw,
                acceleration_raw=acceleration_raw,
            )

    def _read_enable_position_limits(
        self,
        motors: Sequence[str],
    ) -> dict[str, tuple[int, int]]:
        """Read the exact EEPROM limits the servo firmware applies to position goals."""

        return {
            name: (
                int(self.read_register(name, "Min_Position_Limit")),
                int(self.read_register(name, "Max_Position_Limit")),
            )
            for name in motors
        }

    @staticmethod
    def _validate_enable_positions_in_eeprom_limits(
        raw_positions: Mapping[str, int],
        limits: Mapping[str, tuple[int, int]],
    ) -> None:
        """Refuse torque enable if a measured position would be clamped by firmware.

        STS3215 position goals are constrained by the motor's EEPROM Min/Max
        Position Limit registers. Safe enable latches each measured Present_Position
        as Goal_Position before energizing torque, but that is only safe when the
        measured value is already inside the active EEPROM range. Otherwise the
        firmware may clamp the goal to a limit and move abruptly when torque is enabled.
        """

        violations: list[str] = []
        invalid_ranges: list[str] = []
        for name, position in raw_positions.items():
            minimum, maximum = limits[name]
            if minimum >= maximum:
                invalid_ranges.append(
                    f"{name}: invalid EEPROM limits {minimum}..{maximum}"
                )
                continue
            if position < minimum or position > maximum:
                violations.append(
                    f"{name}: present {position} outside EEPROM limits "
                    f"{minimum}..{maximum}"
                )

        if invalid_ranges or violations:
            details = "; ".join((*invalid_ranges, *violations))
            raise SafetyViolationError(
                "refusing torque enable because safe position latching cannot be "
                f"guaranteed: {details}. With torque off, move the affected joint "
                "inside its calibrated range or repair/re-run calibration before enabling."
            )

    def enable_torque(self, motors: Sequence[str] | None = None) -> None:
        with self._io_lock:
            self._require_connected()
            self._require_effort_clear()
            selected = tuple(motors) if motors is not None else ALL_MOTORS
            unknown = set(selected) - set(ALL_MOTORS)
            if unknown:
                raise KeyError(next(iter(unknown)))

            calibration = self._require_calibration()
            uncalibrated = [
                name
                for name in selected
                if name in calibration.uncalibrated_motors
            ]
            if uncalibrated:
                raise SafetyViolationError(
                    "refusing torque enable while selected motors still use factory "
                    "0..4095 calibration ranges: "
                    + ", ".join(uncalibrated)
                    + ". Complete mechanical-stop calibration and reconnect normally first."
                )

            # Read the persistent limits first, then take the final measured-position
            # snapshot immediately before validation/latching. This minimizes the time
            # between the pose we intend to hold and the Goal_Position write.
            limits = self._read_enable_position_limits(selected)
            raw_positions = {name: self.read_raw_position(name) for name in selected}
            self._validate_enable_positions_in_eeprom_limits(raw_positions, limits)

            # Only after all motors pass the precheck do we latch measured positions.
            self._write_raw_positions(raw_positions, speed_raw=1, acceleration_raw=1)

            enabled: list[str] = []
            try:
                for name in selected:
                    # Lock EEPROM before torque is enabled. Normal operation never
                    # needs EEPROM writes and should leave the persistent area protected.
                    self.write_register(name, "Lock", 1)
                    # A missing status packet does not prove the torque write failed.
                    enabled.append(name)
                    self.write_register(name, "Torque_Enable", 1)
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

    def get_hardware_state(self) -> HardwareState:
        """Return hardware state and enforce the latched motor-effort interlock."""

        base = super().get_hardware_state()
        with self._io_lock:
            self._ensure_effort_state()
            effort_message = self._effort_trip_message
            if (
                effort_message is None
                and self._connected
                and self._torque_enabled
                and self._effort_safety_enabled
            ):
                try:
                    effort_message = self._sample_effort_trip()
                except CommunicationError as exc:
                    # A safety channel that cannot be read should fail closed.
                    effort_message = f"motor effort monitor communication failure: {exc}"

                if effort_message is not None:
                    self._effort_trip_message = effort_message
                    try:
                        # Software hold: latch all currently measured positions.
                        super().stop()
                    except Exception as exc:
                        logger.exception("failed to hold arm after effort safety trip")
                        effort_message = f"{effort_message}; software hold failed: {exc}"
                        self._effort_trip_message = effort_message

            faults = [message for message in (base.fault_message, effort_message) if message]
            return HardwareState(
                connected=base.connected,
                torque_enabled=base.torque_enabled,
                moving=False if effort_message else base.moving,
                faulted=base.faulted or effort_message is not None,
                fault_message="; ".join(faults) or None,
            )

    def interactive_calibration(
        self,
        *,
        record_seconds: float = 90.0,
        poll_interval: float = 0.02,
        cancel_event: threading.Event | None = None,
        progress_callback: Callable[[dict[str, dict[str, int | float | bool]]], None]
        | None = None,
    ) -> SO101Calibration:
        """Calibrate all motors from two stop-to-stop traversals per actuator.

        Torque is disabled and the old homing/range values are temporarily reset.
        The user then moves every motor repeatedly through its complete safe range.
        Encoder positions are unwrapped while sampling, so crossing the 4095/0 seam
        is supported. Only after both extrema are known is the midpoint calculated
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
                if cancel_event is not None and cancel_event.is_set():
                    raise CalibrationCancelledError("calibration cancelled")
                # Observe the physical mechanism in raw encoder space. Do not ask
                # the user to guess the midpoint before the true stops are known.
                self.reset_calibration()
                start = self.read_all_raw_positions()
                sweeps = {name: EncoderSweep.start(raw) for name, raw in start.items()}
                display_targets = display_travel_targets(self.config.robot_id)

                deadline = time.monotonic() + record_seconds
                next_progress = 0.0
                if progress_callback is not None:
                    progress_callback(sweep_progress_snapshot(sweeps, display_travel_ticks=display_targets))
                while time.monotonic() < deadline:
                    if cancel_event is not None and cancel_event.is_set():
                        raise CalibrationCancelledError("calibration cancelled")
                    values = self.read_all_raw_positions()
                    for name, raw in values.items():
                        sweeps[name].update(raw)
                    snapshot = sweep_progress_snapshot(
                        sweeps, display_travel_ticks=display_targets
                    )
                    now = time.monotonic()
                    if progress_callback is not None and now >= next_progress:
                        progress_callback(snapshot)
                        next_progress = now + 0.10
                    if all(item["passed"] for item in snapshot.values()):
                        break
                    time.sleep(poll_interval)

                if progress_callback is not None:
                    progress_callback(sweep_progress_snapshot(sweeps, display_travel_ticks=display_targets))
                if cancel_event is not None and cancel_event.is_set():
                    raise CalibrationCancelledError("calibration cancelled")
                calibration = calibration_from_sweeps(sweeps)
                self.apply_calibration(calibration)
                verified = self.read_calibration_from_motors()
                self._verify_calibration_matches_motors(calibration, verified)
                if cancel_event is not None and cancel_event.is_set():
                    raise CalibrationCancelledError("calibration cancelled")
                return calibration
            except BaseException:
                try:
                    self.apply_calibration(eeprom_snapshot)
                    restored = self.read_calibration_from_motors()
                    self._verify_calibration_matches_motors(eeprom_snapshot, restored)
                    self.calibration = previous_calibration or eeprom_snapshot
                except Exception as rollback_exc:
                    raise CalibrationError(
                        "calibration failed and EEPROM rollback also failed; do not enable torque"
                    ) from rollback_exc
                raise
