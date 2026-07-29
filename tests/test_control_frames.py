from __future__ import annotations

from math import pi

import numpy as np
from scipy.spatial.transform import Rotation

from soarm101_motion.control import relative_target_pose
from soarm101_motion.types import Pose


def test_world_translation_follows_base_axes() -> None:
    current = Pose(
        np.array([0.1, 0.2, 0.3]),
        Rotation.from_euler("z", 90, degrees=True).as_matrix(),
    )
    target = relative_target_pose(
        current,
        translation_m=(0.01, 0.0, 0.0),
        frame="world",
    )
    assert np.allclose(target.position, [0.11, 0.2, 0.3])
    assert np.allclose(target.rotation, current.rotation)


def test_tool_translation_follows_current_tcp_axes() -> None:
    current = Pose(
        np.array([0.1, 0.2, 0.3]),
        Rotation.from_euler("z", 90, degrees=True).as_matrix(),
    )
    target = relative_target_pose(
        current,
        translation_m=(0.01, 0.0, 0.0),
        frame="tool",
    )
    assert np.allclose(target.position, [0.1, 0.21, 0.3], atol=1e-12)


def test_world_rotation_pre_multiplies_current_orientation() -> None:
    current = Pose(
        np.zeros(3),
        Rotation.from_euler("x", 30, degrees=True).as_matrix(),
    )
    target = relative_target_pose(
        current,
        rotation_rpy_rad=(0.0, 0.0, pi / 2),
        frame="world",
    )
    expected = (
        Rotation.from_euler("z", 90, degrees=True).as_matrix() @ current.rotation
    )
    assert np.allclose(target.rotation, expected)


def test_tool_rotation_post_multiplies_current_orientation() -> None:
    current = Pose(
        np.zeros(3),
        Rotation.from_euler("x", 30, degrees=True).as_matrix(),
    )
    target = relative_target_pose(
        current,
        rotation_rpy_rad=(0.0, 0.0, pi / 2),
        frame="tool",
    )
    expected = (
        current.rotation @ Rotation.from_euler("z", 90, degrees=True).as_matrix()
    )
    assert np.allclose(target.rotation, expected)
