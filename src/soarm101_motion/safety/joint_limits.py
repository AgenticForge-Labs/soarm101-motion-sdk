"""Joint-target validation.

Default values are conservative provisional software limits and must be verified on hardware.
"""

import math
from collections.abc import Mapping

from soarm101_motion.exceptions import InvalidJointError, SafetyViolationError

DEFAULT_JOINT_LIMITS: dict[str, tuple[float, float]] = {
    "shoulder_pan": (-1.92, 1.92),
    "shoulder_lift": (-1.75, 1.75),
    "elbow_flex": (-1.75, 1.75),
    "wrist_flex": (-1.75, 1.75),
    "wrist_roll": (-2.79, 2.79),
    "gripper": (0.0, 1.0),
}


def validate_joint_targets(
    positions: Mapping[str, float],
    limits: Mapping[str, tuple[float, float]] = DEFAULT_JOINT_LIMITS,
) -> dict[str, float]:
    validated: dict[str, float] = {}
    for name, raw_value in positions.items():
        if name not in limits:
            raise InvalidJointError(f"unknown joint: {name}")
        value = float(raw_value)
        if not math.isfinite(value):
            raise SafetyViolationError(f"joint {name} target must be finite")
        lower, upper = limits[name]
        if not lower <= value <= upper:
            raise SafetyViolationError(
                f"joint {name} target {value} is outside [{lower}, {upper}]"
            )
        validated[name] = value
    if not validated:
        raise SafetyViolationError("at least one joint target is required")
    return validated
