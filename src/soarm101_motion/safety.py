"""Command and workspace safety validation."""

from __future__ import annotations

import math
from collections.abc import Mapping

from soarm101_motion.constants import ARM_JOINTS, JOINT_LIMITS
from soarm101_motion.exceptions import InvalidJointError, SafetyViolationError


def validate_joint_targets(
    targets: Mapping[str, float],
    limits: Mapping[str, tuple[float, float]] = JOINT_LIMITS,
) -> dict[str, float]:
    validated: dict[str, float] = {}
    for name, value in targets.items():
        if name not in ARM_JOINTS:
            raise InvalidJointError(f"unknown arm joint: {name}")
        value = float(value)
        if not math.isfinite(value):
            raise SafetyViolationError(f"joint {name} target is not finite")
        lower, upper = limits[name]
        if not lower <= value <= upper:
            raise SafetyViolationError(
                f"joint {name} target {value:.6f} rad is outside [{lower:.6f}, {upper:.6f}]"
            )
        validated[name] = value
    return validated


def validate_command_step(
    present: Mapping[str, float],
    target: Mapping[str, float],
    max_step_radians: float,
) -> None:
    for name, value in target.items():
        if abs(value - present[name]) > max_step_radians:
            raise SafetyViolationError(
                f"backend command step for {name} exceeds {max_step_radians:.4f} rad; "
                "use the motion controller instead of a raw jump"
            )
