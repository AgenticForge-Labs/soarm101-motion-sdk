from __future__ import annotations

from math import pi

import numpy as np
from scipy.spatial.transform import Rotation

from soarm101_motion.cli.main import build_parser
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
    expected = Rotation.from_euler("z", 90, degrees=True).as_matrix() @ current.rotation
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
    expected = current.rotation @ Rotation.from_euler("z", 90, degrees=True).as_matrix()
    assert np.allclose(target.rotation, expected)


def test_cli_exposes_every_gui_motion_action() -> None:
    parser = build_parser()
    joints = parser.parse_args(
        [
            "move-joints",
            "--port",
            "FAKE",
            "--degrees",
            "0",
            "0",
            "0",
            "0",
            "0",
            "--yes",
        ]
    )
    assert joints.command == "move-joints"

    jog = parser.parse_args(
        [
            "jog",
            "--port",
            "FAKE",
            "--frame",
            "tool",
            "--x-mm",
            "5",
            "--pitch-deg",
            "2",
            "--yes",
        ]
    )
    assert jog.command == "jog"
    assert jog.frame == "tool"
    assert jog.x_mm == 5.0
    assert jog.pitch_deg == 2.0

    gripper = parser.parse_args(["gripper", "--port", "FAKE", "open", "--yes"])
    assert gripper.command == "gripper"
    assert gripper.target == "open"

    gui = parser.parse_args(["gui", "--simulation"])
    assert gui.command == "gui"
    assert gui.simulation is True
