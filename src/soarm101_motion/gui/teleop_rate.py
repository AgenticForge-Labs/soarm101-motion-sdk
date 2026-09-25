"""Rate-limit live leader targets before guarded follower streaming."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

from soarm101_motion.constants import ARM_JOINTS

TELEOP_GRIPPER_SPEED_PER_S = 1.2
GRIPPER_SPEED_PRESETS = (("Slow · original", 1.0), ("Normal · 2×", 2.0), ("Fast · 5×", 5.0))


def gripper_speed_raw(base_speed: int, multiplier: float) -> int:
    """Resolve a GUI speed preset to the Feetech goal speed register."""
    if multiplier not in {value for _, value in GRIPPER_SPEED_PRESETS}:
        raise ValueError("unknown gripper speed preset")
    speed = round(base_speed * multiplier)
    if not 1 <= speed <= 32767:
        raise ValueError("gripper speed exceeds the motor command range")
    return speed


def plan_alignment_target(
    leader: Mapping[str, float],
    motion_limits: Mapping[str, tuple[float, float]],
    calibrated_limits: Mapping[str, tuple[float, float]] | None = None,
) -> tuple[dict[str, float], dict[str, float]]:
    """Stage inside motion limits; accept leader poses within follower calibration.

    Returns the legal follower target and per-joint alignment offsets in radians.
    The offset must be carried into relative teleoperation so the first live
    command cannot jump back outside the motion limits.
    """
    target: dict[str, float] = {}
    offsets: dict[str, float] = {}
    for name in ARM_JOINTS:
        value = float(leader[name])
        lower, upper = motion_limits[name]
        physical_lower, physical_upper = (
            calibrated_limits[name]
            if calibrated_limits is not None and name in calibrated_limits
            else (lower, upper)
        )
        if not math.isfinite(value) or not physical_lower <= value <= physical_upper:
            raise ValueError(
                f"leader {name} is outside the follower's calibrated travel "
                f"[{math.degrees(physical_lower):.1f}°, "
                f"{math.degrees(physical_upper):.1f}°]"
            )
        margin = min(math.radians(0.5), (upper - lower) / 4.0)
        staged = min(max(value, lower + margin), upper - margin)
        target[name] = staged
        if abs(staged - value) > 1e-6:
            offsets[name] = staged - value
    return target, offsets


def _braking_distance(speed: float, acceleration_step: float, period_s: float) -> float:
    """Distance traveled by the remaining discrete deceleration steps."""
    steps = max(0, math.ceil(speed / acceleration_step) - 1)
    return period_s * (
        steps * speed - acceleration_step * steps * (steps + 1) / 2
    )


def limit_joint_target(
    desired: Mapping[str, float],
    previous: Mapping[str, float],
    previous_velocity: Mapping[str, float],
    *,
    joint_limits: Mapping[str, tuple[float, float]] | None = None,
    period_s: float,
    max_speed_rad_s: float,
    max_acceleration_rad_s2: float,
    max_step_rad: float,
) -> tuple[dict[str, float], dict[str, float], bool]:
    """Return a target whose step, speed, and acceleration fit one stream period."""
    speed_ceiling = min(max_speed_rad_s, max_step_rad / period_s)
    command: dict[str, float] = {}
    velocity: dict[str, float] = {}
    limited = False
    for name in ARM_JOINTS:
        wanted_position = desired[name]
        if joint_limits is not None:
            lower, upper = joint_limits[name]
            bounded_position = min(max(wanted_position, lower), upper)
            limited = limited or bounded_position != wanted_position
            wanted_position = bounded_position
        error = wanted_position - previous[name]
        if abs(error) < 1e-10:
            wanted = 0.0
        else:
            # Leave enough distance to decelerate on later stream samples. A
            # plain position/speed clamp can reach the target with nonzero
            # velocity, then command past it and reverse on the next sample.
            low = 0.0
            high = min(speed_ceiling, abs(error) / period_s)
            for _ in range(32):
                candidate = (low + high) / 2
                travel = candidate * period_s + _braking_distance(
                    candidate, max_acceleration_rad_s2 * period_s, period_s
                )
                if travel <= abs(error):
                    low = candidate
                else:
                    high = candidate
            wanted = math.copysign(low, error)
        last_velocity = previous_velocity[name]
        acceleration_step = max_acceleration_rad_s2 * period_s
        bounded = max(last_velocity - acceleration_step, min(last_velocity + acceleration_step, wanted))
        bounded = max(-speed_ceiling, min(speed_ceiling, bounded))
        command[name] = previous[name] + bounded * period_s
        velocity[name] = bounded
        limited = limited or abs(command[name] - wanted_position) > 1e-8
    return command, velocity, limited


@dataclass
class GripperContactLatch:
    """Track a teleoperated gripper and latch its aperture when closing stalls."""

    last_command: float
    last_actual: float
    contact_position: float | None = None
    stalled_samples: int = 0
    stall_start_actual: float | None = None
    contact_leader_target: float | None = None
    last_desired: float | None = None
    opening_after_release: bool = False
    opening_peak_desired: float | None = None


def update_gripper_contact_latch(
    state: GripperContactLatch,
    desired: float,
    actual: float,
    *,
    period_s: float,
    max_speed_per_s: float = TELEOP_GRIPPER_SPEED_PER_S,
    contact_gap: float = 0.025,
    movement_epsilon: float = 0.01,
    stalled_samples_required: int = 6,
    release_margin: float = 0.04,
    minimum_opening: float = 0.025,
    contact_relief: float = 0.005,
) -> tuple[float, bool, bool, bool]:
    """Rate-limit closing and ease open slightly after contact.

    Gripper positions use 0=closed and 1=open. Returns the target, whether
    contact is latched, whether contact was newly detected, and whether a
    previous latch was released by opening the leader gripper.
    """
    values = (desired, actual, period_s, max_speed_per_s)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("gripper teleop values must be finite")
    if period_s <= 0 or max_speed_per_s <= 0:
        raise ValueError("gripper teleop period and speed must be positive")
    if contact_gap < 0 or movement_epsilon < 0 or release_margin < 0:
        raise ValueError("gripper contact thresholds cannot be negative")
    if not 0.0 <= minimum_opening <= 1.0 or not 0.0 <= contact_relief <= 1.0:
        raise ValueError("gripper opening margins must be within [0, 1]")
    if stalled_samples_required < 1:
        raise ValueError("stalled_samples_required must be positive")

    desired = min(1.0, max(0.0, desired))
    leader_desired = desired
    actual = min(1.0, max(0.0, actual))
    released = False
    newly_latched = False

    previous_desired = state.last_desired
    state.last_desired = desired
    if state.contact_position is not None:
        # The leader and follower can have different calibrated apertures. A
        # threshold based on follower contact may be unreachable by the leader.
        release_target = min(1.0, float(state.contact_leader_target) + release_margin)
        if desired < release_target:
            state.last_actual = actual
            state.last_command = state.contact_position
            return state.contact_position, True, False, False
        state.contact_position = None
        state.contact_leader_target = None
        state.stalled_samples = 0
        state.stall_start_actual = None
        state.opening_after_release = True
        state.opening_peak_desired = desired
        released = True

    max_step = max_speed_per_s * period_s
    if state.opening_after_release:
        state.opening_peak_desired = max(float(state.opening_peak_desired), desired)
        if desired < state.opening_peak_desired - 0.02:
            state.opening_after_release = False
            state.opening_peak_desired = None
        else:
            opening_delta = max(
                0.0, desired - (previous_desired if previous_desired is not None else desired)
            )
            desired = max(desired, state.last_command + opening_delta)
    desired = max(minimum_opening, min(1.0, desired))
    command = min(state.last_command + max_step, max(state.last_command - max_step, desired))
    closing_into_contact = command < actual - contact_gap and not state.opening_after_release
    if closing_into_contact:
        if (
            state.stall_start_actual is None
            or abs(actual - state.stall_start_actual) >= movement_epsilon
        ):
            state.stall_start_actual = actual
            state.stalled_samples = 1
        elif (
            previous_desired is not None
            and leader_desired > previous_desired + release_margin / 2
        ):
            state.stall_start_actual = None
            state.stalled_samples = 0
        else:
            state.stalled_samples += 1
    else:
        state.stalled_samples = 0
        state.stall_start_actual = None

    if state.stalled_samples >= stalled_samples_required:
        hold_position = max(minimum_opening, min(1.0, actual + contact_relief))
        state.contact_position = hold_position
        state.contact_leader_target = leader_desired
        state.last_command = hold_position
        state.last_actual = actual
        state.stall_start_actual = None
        newly_latched = True
        return hold_position, True, newly_latched, released

    state.last_command = command
    state.last_actual = actual
    return command, False, newly_latched, released
