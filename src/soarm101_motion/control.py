"""Shared control actions used by the CLI and PySide6 GUI."""

from __future__ import annotations

from collections.abc import Sequence
from math import pi
from typing import Literal

import numpy as np
from scipy.spatial.transform import Rotation

from soarm101_motion.arm import SOARM101
from soarm101_motion.kinematics import OrientationMode
from soarm101_motion.motion import MotionHandle
from soarm101_motion.types import MotionResult, Pose

ControlFrame = Literal["world", "tool"]


def relative_target_pose(
    current: Pose,
    *,
    translation_m: Sequence[float] = (0.0, 0.0, 0.0),
    rotation_rpy_rad: Sequence[float] = (0.0, 0.0, 0.0),
    frame: ControlFrame = "world",
) -> Pose:
    """Apply a small relative transform from the world or current tool frame.

    ``world`` translations follow the base axes and rotations occur around world
    X/Y/Z at the current TCP position. ``tool`` translations and rotations follow
    the current TCP axes. The pose is returned in the robot base/world frame.
    """

    translation = np.asarray(tuple(translation_m), dtype=float).reshape(3)
    rotation_delta = Rotation.from_euler(
        "xyz", np.asarray(tuple(rotation_rpy_rad), dtype=float).reshape(3)
    ).as_matrix()

    if frame == "world":
        return Pose(
            current.position + translation,
            rotation_delta @ current.rotation,
        )
    if frame == "tool":
        return Pose(
            current.position + current.rotation @ translation,
            current.rotation @ rotation_delta,
        )
    raise ValueError("frame must be 'world' or 'tool'")


def jog_linear(
    arm: SOARM101,
    *,
    frame: ControlFrame,
    translation_m: Sequence[float] = (0.0, 0.0, 0.0),
    rotation_rpy_rad: Sequence[float] = (0.0, 0.0, 0.0),
    orientation_mode: OrientationMode = "compatible",
    speed_m_s: float | None = None,
    acceleration_m_s2: float | None = None,
    wait: bool = True,
) -> MotionResult | MotionHandle[MotionResult]:
    current = arm.get_position()
    target = relative_target_pose(
        current,
        translation_m=translation_m,
        rotation_rpy_rad=rotation_rpy_rad,
        frame=frame,
    )
    return arm.move_linear(
        target,
        orientation_mode=orientation_mode,
        speed=speed_m_s,
        acceleration=acceleration_m_s2,
        wait=wait,
    )


def jog_linear_cli_units(
    arm: SOARM101,
    *,
    frame: ControlFrame,
    translation_mm: Sequence[float] = (0.0, 0.0, 0.0),
    rotation_rpy_deg: Sequence[float] = (0.0, 0.0, 0.0),
    orientation_mode: OrientationMode = "compatible",
    speed_mm_s: float | None = None,
    acceleration_mm_s2: float | None = None,
    wait: bool = True,
) -> MotionResult | MotionHandle[MotionResult]:
    return jog_linear(
        arm,
        frame=frame,
        translation_m=np.asarray(tuple(translation_mm), dtype=float) / 1000.0,
        rotation_rpy_rad=np.asarray(tuple(rotation_rpy_deg), dtype=float) * pi / 180.0,
        orientation_mode=orientation_mode,
        speed_m_s=None if speed_mm_s is None else speed_mm_s / 1000.0,
        acceleration_m_s2=(
            None if acceleration_mm_s2 is None else acceleration_mm_s2 / 1000.0
        ),
        wait=wait,
    )
