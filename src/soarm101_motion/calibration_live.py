"""Live mechanical-stop calibration helpers.

The SO-ARM101 printed structure provides repeatable mechanical end stops.  During
calibration the unpowered arm can therefore be swept through its full travel and
the encoder extrema can be used to define the physical midpoint.  This module
keeps that logic independent from the serial transport so it can be tested
without hardware.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import ceil
from typing import Mapping

from soarm101_motion.calibration import (
    MotorCalibration,
    SO101Calibration,
    default_calibration_path,
)
from soarm101_motion.constants import (
    ALL_MOTORS,
    ARM_JOINTS,
    ENCODER_RESOLUTION,
    HALF_TURN,
    MOTOR_IDS,
    STOCK_GRIPPER,
)
from soarm101_motion.exceptions import CalibrationError

# Provisional lower bounds for a credible stop-to-stop sweep.  The five pose joints
# are known mechanically to exceed half an encoder revolution, so require at least
# 180 degrees / 2048 ticks. Measured leader and follower grippers span about 1188
# and 1488 ticks respectively; 900 ticks leaves assembly margin without accepting
# a tiny partial sweep.
PROVISIONAL_MINIMUM_TRAVEL_TICKS: dict[str, int] = {
    **{name: ENCODER_RESOLUTION // 2 for name in ARM_JOINTS},
    STOCK_GRIPPER: 900,
}
# Visual gauge endpoints based on the first measured follower sweep (2026-09-23).
# They are display targets only: the provisional acceptance minimum above still
# determines whether a calibration may be saved. Wrist roll traversed 3854 ticks,
# so its gauge should not appear full halfway through the physical rotation.
SWEEP_DISPLAY_TRAVEL_TICKS: dict[str, int] = {
    "shoulder_pan": 2200,
    "shoulder_lift": 2250,
    "elbow_flex": 2150,
    "wrist_flex": 2250,
    "wrist_roll": 3600,
    STOCK_GRIPPER: 1100,
}
REQUIRED_FULL_TRAVERSALS = 2
REVERSAL_HYSTERESIS_TICKS = 24
MINIMUM_CALIBRATION_TRAVEL_TICKS = min(PROVISIONAL_MINIMUM_TRAVEL_TICKS.values())
MAXIMUM_HOMING_OFFSET = (ENCODER_RESOLUTION // 2) - 1


def display_travel_targets(robot_id: str | None = None) -> dict[str, int]:
    """Use a prior local calibration to scale gauges, without changing legality."""
    targets = dict(SWEEP_DISPLAY_TRAVEL_TICKS)
    if not robot_id:
        return targets
    try:
        prior = SO101Calibration.load(default_calibration_path(robot_id))
    except CalibrationError:
        return targets
    for name in ALL_MOTORS:
        motor = prior.motors.get(name)
        if motor is None:
            continue
        span = motor.range_max - motor.range_min
        if PROVISIONAL_MINIMUM_TRAVEL_TICKS[name] <= span < ENCODER_RESOLUTION:
            targets[name] = max(
                PROVISIONAL_MINIMUM_TRAVEL_TICKS[name], round(0.95 * span)
            )
    return targets


def minimum_travel_targets(
    override: int | Mapping[str, int] | None = None,
) -> dict[str, int]:
    """Return per-motor minimum sweep spans used for pass/fail and UI progress."""

    if override is None:
        return dict(PROVISIONAL_MINIMUM_TRAVEL_TICKS)
    if isinstance(override, int):
        if override <= 0:
            raise ValueError("minimum calibration travel must be positive")
        return {name: int(override) for name in ALL_MOTORS}
    targets = dict(PROVISIONAL_MINIMUM_TRAVEL_TICKS)
    for name, value in override.items():
        if name not in ALL_MOTORS:
            raise KeyError(name)
        ticks = int(value)
        if ticks <= 0:
            raise ValueError(f"minimum calibration travel for {name} must be positive")
        targets[name] = ticks
    return targets


def sweep_progress_snapshot(
    sweeps: Mapping[str, "EncoderSweep"],
    *,
    minimum_travel_ticks: int | Mapping[str, int] | None = None,
    display_travel_ticks: Mapping[str, int] | None = None,
) -> dict[str, dict[str, int | float | bool]]:
    """Return immutable UI/test-friendly progress for the current live sweep."""

    targets = minimum_travel_targets(minimum_travel_ticks)
    result: dict[str, dict[str, int | float | bool]] = {}
    for name in ALL_MOTORS:
        sweep = sweeps.get(name)
        travel = 0 if sweep is None else sweep.travel_ticks
        required = targets[name]
        display = max(required, (display_travel_ticks or SWEEP_DISPLAY_TRAVEL_TICKS)[name])
        completed = 0 if sweep is None else sweep.completed_traversals(required)
        stage = min(REQUIRED_FULL_TRAVERSALS, 1 + (0 if sweep is None else sweep.finalized_traversals(required)))
        leg = 0 if sweep is None else sweep.active_leg_ticks
        best_leg = 0 if sweep is None else sweep.best_current_leg_ticks(required)
        result[name] = {
            "travel_ticks": int(travel),
            "required_ticks": int(required),
            "display_ticks": int(display),
            "fraction": 1.0 if completed >= REQUIRED_FULL_TRAVERSALS else min(1.0, float(best_leg) / float(display)),
            "active_leg_ticks": int(leg),
            "best_leg_ticks": int(best_leg),
            "sweep_number": stage,
            "traversals_completed": min(REQUIRED_FULL_TRAVERSALS, completed),
            "passed": bool(travel >= required and completed >= REQUIRED_FULL_TRAVERSALS),
            "samples": 0 if sweep is None else int(sweep.samples),
        }
    return result


@dataclass
class EncoderSweep:
    """Track one encoder through a live sweep, including 4095 -> 0 crossings."""

    last_raw: int
    unwrapped: int
    minimum: int
    maximum: int
    samples: int = 1
    seam_crossings: int = 0
    leg_origin: int = 0
    leg_extreme: int = 0
    leg_direction: int = 0
    finished_leg_ticks: list[int] = field(default_factory=list)

    @classmethod
    def start(cls, raw: int) -> "EncoderSweep":
        value = int(raw) % ENCODER_RESOLUTION
        return cls(value, value, value, value, leg_origin=value, leg_extreme=value)

    def update(self, raw: int) -> None:
        value = int(raw) % ENCODER_RESOLUTION
        delta = value - self.last_raw
        half = ENCODER_RESOLUTION // 2
        if delta > half:
            delta -= ENCODER_RESOLUTION
            self.seam_crossings += 1
        elif delta < -half:
            delta += ENCODER_RESOLUTION
            self.seam_crossings += 1
        self.unwrapped += delta
        self.last_raw = value
        if self.leg_direction == 0:
            offset = self.unwrapped - self.leg_origin
            if abs(offset) >= REVERSAL_HYSTERESIS_TICKS:
                self.leg_direction = 1 if offset > 0 else -1
                self.leg_extreme = self.unwrapped
        elif self.leg_direction > 0:
            if self.unwrapped > self.leg_extreme:
                self.leg_extreme = self.unwrapped
            elif self.leg_extreme - self.unwrapped >= REVERSAL_HYSTERESIS_TICKS:
                self.finished_leg_ticks.append(self.active_leg_ticks)
                self.leg_origin = self.leg_extreme
                self.leg_extreme = self.unwrapped
                self.leg_direction = -1
        else:
            if self.unwrapped < self.leg_extreme:
                self.leg_extreme = self.unwrapped
            elif self.unwrapped - self.leg_extreme >= REVERSAL_HYSTERESIS_TICKS:
                self.finished_leg_ticks.append(self.active_leg_ticks)
                self.leg_origin = self.leg_extreme
                self.leg_extreme = self.unwrapped
                self.leg_direction = 1
        self.minimum = min(self.minimum, self.unwrapped)
        self.maximum = max(self.maximum, self.unwrapped)
        self.samples += 1

    @property
    def travel_ticks(self) -> int:
        return self.maximum - self.minimum

    @property
    def active_leg_ticks(self) -> int:
        return abs(self.leg_extreme - self.leg_origin)

    def _full_traversal_threshold(self, required_ticks: int) -> int:
        return max(int(required_ticks), ceil(0.95 * self.travel_ticks))

    def finalized_traversals(self, required_ticks: int) -> int:
        threshold = self._full_traversal_threshold(required_ticks)
        return sum(length >= threshold for length in self.finished_leg_ticks)

    def completed_traversals(self, required_ticks: int) -> int:
        threshold = self._full_traversal_threshold(required_ticks)
        return self.finalized_traversals(required_ticks) + int(self.active_leg_ticks >= threshold)

    def best_current_leg_ticks(self, required_ticks: int) -> int:
        """Keep the best partial attempt visible until this stage completes."""
        threshold = self._full_traversal_threshold(required_ticks)
        best = 0
        for length in self.finished_leg_ticks:
            best = 0 if length >= threshold else max(best, length)
        return max(best, self.active_leg_ticks)

    @property
    def center_unwrapped(self) -> float:
        return (self.minimum + self.maximum) / 2.0


def _signed_homing_offset(center_raw: int) -> int:
    """Return the shortest Feetech homing offset mapping center to HALF_TURN."""

    half = ENCODER_RESOLUTION // 2
    offset = ((int(center_raw) - HALF_TURN + half) % ENCODER_RESOLUTION) - half
    if abs(offset) > MAXIMUM_HOMING_OFFSET:
        raise CalibrationError(
            f"mechanical midpoint requires unsupported homing offset {offset}; "
            "re-seat the servo horn closer to its centered encoder position"
        )
    return offset


def calibration_from_sweeps(
    sweeps: Mapping[str, EncoderSweep],
    *,
    minimum_travel_ticks: int | Mapping[str, int] | None = None,
) -> SO101Calibration:
    """Build motor calibration after two full live traversals per motor.

    The sweep is recorded with homing offset zero.  Each motor's physical zero is
    defined as exactly halfway between the observed mechanical extrema.  The
    resulting homing offset places that midpoint at ``HALF_TURN`` and the stored
    limits are symmetric around it.  Because extrema are tracked in unwrapped
    encoder coordinates, a joint may safely cross the 4095/0 seam during the
    calibration sweep.
    """

    missing = set(ALL_MOTORS) - set(sweeps)
    if missing:
        raise CalibrationError(f"calibration sweep is missing motors: {sorted(missing)}")

    targets = minimum_travel_targets(minimum_travel_ticks)

    motors: dict[str, MotorCalibration] = {}
    for name in ALL_MOTORS:
        sweep = sweeps[name]
        travel = sweep.travel_ticks
        required = targets[name]
        if travel < required:
            raise CalibrationError(
                f"{name} moved only {travel} encoder ticks; at least {required} ticks "
                "are required by the current provisional sweep threshold; sweep it "
                "repeatedly between both mechanical stops and recalibrate."
            )
        if travel >= ENCODER_RESOLUTION:
            raise CalibrationError(
                f"{name} appears to have moved {travel} ticks (one full encoder turn or more); "
                "the stock printed SO-ARM101 should be bounded by mechanical stops"
            )
        completed = sweep.completed_traversals(required)
        if completed < REQUIRED_FULL_TRAVERSALS:
            raise CalibrationError(
                f"{name} completed {completed}/{REQUIRED_FULL_TRAVERSALS} full "
                "end-to-end traversals; move between both stops twice and recalibrate."
            )
        center_raw = int(round(sweep.center_unwrapped)) % ENCODER_RESOLUTION
        homing_offset = _signed_homing_offset(center_raw)

        # Once the midpoint is shifted to HALF_TURN, the two observed extrema are
        # symmetric by construction.  Deriving the limits this way avoids any
        # ambiguity when the raw sweep crossed the encoder seam.
        lower = int(round(HALF_TURN - travel / 2.0))
        upper = int(round(HALF_TURN + travel / 2.0))
        lower = max(0, lower)
        upper = min(ENCODER_RESOLUTION - 1, upper)
        if lower >= upper:
            raise CalibrationError(f"{name} produced invalid calibrated limits {lower}..{upper}")

        motors[name] = MotorCalibration(
            motor_id=MOTOR_IDS[name],
            drive_mode=0,
            homing_offset=homing_offset,
            range_min=lower,
            range_max=upper,
        )

    calibration = SO101Calibration(
        motors=motors,
        source="live-mechanical-stop-midpoint",
    )
    calibration.validate()
    return calibration
