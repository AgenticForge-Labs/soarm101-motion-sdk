"""Live mechanical-stop calibration helpers.

The SO-ARM101 printed structure provides repeatable mechanical end stops.  During
calibration the unpowered arm can therefore be swept through its full travel and
the encoder extrema can be used to define the physical midpoint.  This module
keeps that logic independent from the serial transport so it can be tested
without hardware.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from soarm101_motion.calibration import MotorCalibration, SO101Calibration
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
# 180 degrees / 2048 ticks.  The gripper travel is shorter and remains deliberately
# conservative until we characterize a physical arm.  Keep these values centralized:
# once measured stop-to-stop spans are collected, only this table and its associated
# UI wording need to change.
PROVISIONAL_MINIMUM_TRAVEL_TICKS: dict[str, int] = {
    **{name: ENCODER_RESOLUTION // 2 for name in ARM_JOINTS},
    STOCK_GRIPPER: 256,
}
MINIMUM_CALIBRATION_TRAVEL_TICKS = min(PROVISIONAL_MINIMUM_TRAVEL_TICKS.values())
MAXIMUM_HOMING_OFFSET = (ENCODER_RESOLUTION // 2) - 1


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
) -> dict[str, dict[str, int | float | bool]]:
    """Return immutable UI/test-friendly progress for the current live sweep."""

    targets = minimum_travel_targets(minimum_travel_ticks)
    result: dict[str, dict[str, int | float | bool]] = {}
    for name in ALL_MOTORS:
        sweep = sweeps.get(name)
        travel = 0 if sweep is None else sweep.travel_ticks
        required = targets[name]
        result[name] = {
            "travel_ticks": int(travel),
            "required_ticks": int(required),
            "fraction": min(1.0, float(travel) / float(required)),
            "passed": bool(travel >= required),
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

    @classmethod
    def start(cls, raw: int) -> "EncoderSweep":
        value = int(raw) % ENCODER_RESOLUTION
        return cls(value, value, value, value)

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
        self.minimum = min(self.minimum, self.unwrapped)
        self.maximum = max(self.maximum, self.unwrapped)
        self.samples += 1

    @property
    def travel_ticks(self) -> int:
        return self.maximum - self.minimum

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
    """Build motor calibration from complete live stop-to-stop sweeps.

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
                "are required by the current provisional sweep threshold. Sweep it "
                "repeatedly between both mechanical stops and recalibrate."
            )
        if travel >= ENCODER_RESOLUTION:
            raise CalibrationError(
                f"{name} appears to have moved {travel} ticks (one full encoder turn or more); "
                "the stock printed SO-ARM101 should be bounded by mechanical stops"
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
