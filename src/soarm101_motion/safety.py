"""Command and conservative workspace safety validation."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

from soarm101_motion.constants import ARM_JOINTS, JOINT_LIMITS
from soarm101_motion.exceptions import InvalidJointError, SafetyViolationError

if TYPE_CHECKING:
    from soarm101_motion.kinematics.model import SO101KinematicModel
    from soarm101_motion.types import Pose

FloatArray = NDArray[np.float64]


def resolve_effective_joint_limits(
    calibrated_limits: Mapping[str, tuple[float, float]] | None,
    *,
    calibration_extension_stop_margin_rad: float,
    model_limits: Mapping[str, tuple[float, float]] = JOINT_LIMITS,
) -> dict[str, tuple[float, float]]:
    """Resolve executable pose-joint limits from model and measured calibration.

    The nominal model range remains available whenever it overlaps the measured
    mechanical range. Calibration may extend that nominal authority only where the
    measured stop-to-stop range leaves the configured margin before the physical stop.
    The margin never narrows the pre-existing model/calibration intersection.

    This keeps nominal model limits as the baseline while allowing arm-specific
    calibration to prove conservative extra travel without ever commanding the
    measured mechanical stop itself.
    """

    margin = float(calibration_extension_stop_margin_rad)
    if not math.isfinite(margin) or margin <= 0.0:
        raise ValueError(
            "calibration_extension_stop_margin_rad must be positive and finite"
        )

    resolved = {
        name: (float(model_limits[name][0]), float(model_limits[name][1]))
        for name in ARM_JOINTS
    }
    if calibrated_limits is None:
        return resolved

    for name in ARM_JOINTS:
        if name not in calibrated_limits:
            continue
        model_lower, model_upper = resolved[name]
        calibrated_lower, calibrated_upper = (
            float(calibrated_limits[name][0]),
            float(calibrated_limits[name][1]),
        )
        if (
            not math.isfinite(calibrated_lower)
            or not math.isfinite(calibrated_upper)
            or calibrated_lower >= calibrated_upper
        ):
            raise SafetyViolationError(f"invalid calibrated joint range for {name}")

        normal_lower = max(model_lower, calibrated_lower)
        normal_upper = min(model_upper, calibrated_upper)
        if normal_lower >= normal_upper:
            raise SafetyViolationError(
                f"calibrated range for {name} does not overlap the model limits"
            )

        inset_lower = calibrated_lower + margin
        inset_upper = calibrated_upper - margin

        # Never narrow today's normal model/calibration intersection. Extend only
        # when measured calibration leaves the configured margin beyond it.
        lower = normal_lower
        upper = normal_upper
        if inset_lower < normal_lower:
            lower = inset_lower
        if inset_upper > normal_upper:
            upper = inset_upper

        # Calibration remains the physical authority.
        lower = max(lower, calibrated_lower)
        upper = min(upper, calibrated_upper)
        if lower >= upper:
            raise SafetyViolationError(f"resolved joint range for {name} is empty")
        resolved[name] = (lower, upper)

    return resolved


def validate_joint_targets(    targets: Mapping[str, float],
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


def _segment_distance(a0: FloatArray, a1: FloatArray, b0: FloatArray, b1: FloatArray) -> float:
    """Shortest distance between two finite 3-D line segments."""
    u = a1 - a0
    v = b1 - b0
    w = a0 - b0
    aa = float(np.dot(u, u))
    bb = float(np.dot(u, v))
    cc = float(np.dot(v, v))
    dd = float(np.dot(u, w))
    ee = float(np.dot(v, w))
    denominator = aa * cc - bb * bb
    epsilon = 1e-12

    if aa < epsilon and cc < epsilon:
        return float(np.linalg.norm(a0 - b0))
    if aa < epsilon:
        t = min(1.0, max(0.0, ee / cc))
        return float(np.linalg.norm(a0 - (b0 + t * v)))
    if cc < epsilon:
        s = min(1.0, max(0.0, -dd / aa))
        return float(np.linalg.norm((a0 + s * u) - b0))

    if denominator < epsilon:
        s = 0.0
        t = min(1.0, max(0.0, ee / cc))
    else:
        s = min(1.0, max(0.0, (bb * ee - cc * dd) / denominator))
        t = (bb * s + ee) / cc
        if t < 0.0:
            t = 0.0
            s = min(1.0, max(0.0, -dd / aa))
        elif t > 1.0:
            t = 1.0
            s = min(1.0, max(0.0, (bb - dd) / aa))

    closest_a = a0 + s * u
    closest_b = b0 + t * v
    return float(np.linalg.norm(closest_a - closest_b))


def validate_workspace_configuration(
    model: "SO101KinematicModel",
    joints: Mapping[str, float],
    *,
    tcp: "Pose | None" = None,
    minimum_z_m: float = 0.0,
    maximum_tcp_reach_m: float = 0.50,
    minimum_self_clearance_m: float = 0.025,
    base_keepout_radius_m: float = 0.055,
    base_keepout_height_m: float = 0.11,
) -> None:
    """Reject gross floor, base, reach, and centerline self-collision hazards.

    This intentionally uses a coarse centerline model. It catches obvious fold-back
    and table/base intersections but is not a mesh-level collision checker.
    """
    points = model.link_points(joints, tcp=tcp)
    ordered_names = tuple(points)
    ordered = tuple(points.values())

    for name in ("elbow_flex", "wrist_flex", "wrist_roll", "tcp"):
        point = points[name]
        if float(point[2]) < minimum_z_m:
            raise SafetyViolationError(
                f"workspace check: {name} z={point[2]:.3f} m is below {minimum_z_m:.3f} m"
            )

    tcp_point = points["tcp"]
    tcp_reach = float(np.linalg.norm(tcp_point))
    if tcp_reach > maximum_tcp_reach_m:
        raise SafetyViolationError(
            f"workspace check: TCP reach {tcp_reach:.3f} m exceeds {maximum_tcp_reach_m:.3f} m"
        )

    for name in ("elbow_flex", "wrist_flex", "wrist_roll", "tcp"):
        point = points[name]
        radial = float(np.hypot(point[0], point[1]))
        if minimum_z_m <= point[2] <= base_keepout_height_m and radial < base_keepout_radius_m:
            raise SafetyViolationError(
                f"workspace check: {name} enters the coarse base keep-out cylinder"
            )

    segments = tuple(zip(ordered[:-1], ordered[1:], strict=True))
    for first_index, (first_start, first_end) in enumerate(segments):
        for second_index in range(first_index + 2, len(segments)):
            # Neighboring links meet at a joint and are intentionally ignored.
            if second_index - first_index <= 1:
                continue
            second_start, second_end = segments[second_index]
            clearance = _segment_distance(first_start, first_end, second_start, second_end)
            if clearance < minimum_self_clearance_m:
                first_name = f"{ordered_names[first_index]}->{ordered_names[first_index + 1]}"
                second_name = f"{ordered_names[second_index]}->{ordered_names[second_index + 1]}"
                raise SafetyViolationError(
                    "workspace check: coarse self-clearance between "
                    f"{first_name} and {second_name} is {clearance:.3f} m, below "
                    f"{minimum_self_clearance_m:.3f} m"
                )


def validate_workspace_path(
    model: "SO101KinematicModel",
    samples: Sequence[Mapping[str, float]],
    *,
    tcp: "Pose | None" = None,
    minimum_z_m: float = 0.0,
    maximum_tcp_reach_m: float = 0.50,
    minimum_self_clearance_m: float = 0.025,
    base_keepout_radius_m: float = 0.055,
    base_keepout_height_m: float = 0.11,
) -> None:
    for index, joints in enumerate(samples):
        try:
            validate_workspace_configuration(
                model,
                joints,
                tcp=tcp,
                minimum_z_m=minimum_z_m,
                maximum_tcp_reach_m=maximum_tcp_reach_m,
                minimum_self_clearance_m=minimum_self_clearance_m,
                base_keepout_radius_m=base_keepout_radius_m,
                base_keepout_height_m=base_keepout_height_m,
            )
        except SafetyViolationError as exc:
            raise SafetyViolationError(f"workspace path sample {index}: {exc}") from exc
