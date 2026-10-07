"""URDF parity and stock jaw behavior independent of the GUI projection."""

from importlib.resources import files
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from soarm101_motion.kinematics import SO101KinematicModel
from soarm101_motion.kinematics.model import (
    SO101_PRESENTATION_BOXES,
    STOCK_JAW_JOINT,
    STOCK_JAW_LIMITS,
)
from soarm101_motion.types import Pose


def _origin(element_parent):
    element = element_parent.find("origin")
    frame = np.eye(4)
    frame[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
    frame[:3, :3] = Rotation.from_euler(
        "xyz", np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
    ).as_matrix()
    return frame


def test_native_fk_matches_packaged_urdf_at_varied_configurations():
    root = ET.fromstring(files("soarm101_motion.models").joinpath("so101.urdf").read_text())
    model = SO101KinematicModel()
    for q in np.random.default_rng(101).uniform(model.lower_bounds, model.upper_bounds, (100, 5)):
        frame = np.eye(4)
        for name, angle in zip(model.joint_names, q, strict=True):
            joint = root.find(f"joint[@name='{name}']")
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
            frame = frame @ _origin(joint) @ rotation
        frame = frame @ _origin(root.find("joint[@name='tcp_joint']"))
        # Packaged RPY values are rounded to eight decimal places.
        assert model.forward_matrix(q) == pytest.approx(frame, abs=1e-7)
    jaw = root.find("joint[@name='gripper']")
    assert STOCK_JAW_JOINT.origin_transform == pytest.approx(_origin(jaw), abs=1e-8)
    assert STOCK_JAW_LIMITS == pytest.approx(
        (float(jaw.find("limit").get("lower")), float(jaw.find("limit").get("upper")))
    )


def test_presentation_members_preserve_off_axis_mesh_placement():
    model = SO101KinematicModel()
    joints = np.array([0.2, -0.35, 0.5, -0.4, 0.3])
    frames = model.link_frames(joints)
    segments = model.presentation_link_segments(joints)

    expected_local = {
        "upper_arm": (
            "upper_arm_link",
            np.array([-0.065085, 0.012, 0.0182]),
        ),
        "lower_arm": (
            "lower_arm_link",
            np.array([-0.06485, -0.032, 0.0182]),
        ),
    }
    for name, (frame_name, expected_center) in expected_local.items():
        first, second = segments[name]
        center_world = (first + second) / 2.0
        frame = frames[frame_name]
        center_local = frame[:3, :3].T @ (center_world - frame[:3, 3])
        assert center_local == pytest.approx(expected_center, abs=1e-7)

    # The presentation must not silently collapse the printed arms back onto the
    # kinematic X axis. Their lateral/vertical offsets are physical-model evidence.
    assert expected_local["upper_arm"][1][1] != 0.0
    assert expected_local["lower_arm"][1][1] != 0.0


def test_mesh_informed_motor_envelopes_contain_their_joint_axes():
    model = SO101KinematicModel()
    joints = np.array([0.2, -0.35, 0.5, -0.4, 0.3])
    frames = model.link_frames(joints)
    joint_points = model.link_points(joints)
    driven_joint = {
        "shoulder_pan_motor": "shoulder_pan",
        "shoulder_lift_motor": "shoulder_lift",
        "elbow_flex_motor": "elbow_flex",
        "wrist_flex_motor": "wrist_flex",
        "wrist_roll_motor": "wrist_roll",
    }

    definitions = {
        definition.name: definition for definition in SO101_PRESENTATION_BOXES
    }
    corners = model.presentation_box_corners(joints)
    assert set(driven_joint).issubset(corners)
    assert all(len(points) == 8 for points in corners.values())

    for motor_name, joint_name in driven_joint.items():
        definition = definitions[motor_name]
        box_frame = frames[definition.frame] @ definition.local_transform
        local_joint = box_frame[:3, :3].T @ (
            joint_points[joint_name] - box_frame[:3, 3]
        )
        half_size = np.asarray(definition.size_local) / 2.0
        assert np.all(np.abs(local_joint) <= half_size + 1e-8), (
            motor_name,
            joint_name,
            local_joint,
            half_size,
        )


def test_stock_jaw_rotates_about_fixed_pivot_and_ignores_active_tool_tcp():
    model = SO101KinematicModel()
    joints = np.array([0.3, -0.4, 0.7, 0.2, -0.3])
    closed = model.stock_gripper_points(joints, 0.0)
    for opening in (0.25, 0.5, 0.75, 1.0):
        points = model.stock_gripper_points(joints, opening)
        assert points["fixed_tip"] == pytest.approx(closed["fixed_tip"])
        assert points["moving_pivot"] == pytest.approx(closed["moving_pivot"])
        assert np.linalg.norm(points["moving_tip"] - points["moving_pivot"]) == pytest.approx(
            np.linalg.norm(closed["moving_tip"] - closed["moving_pivot"])
        )
    model.tcp = Pose.from_xyz_rpy(0.05, 0.01, -0.15, 0, 0, 0)
    for name, point in model.stock_gripper_points(joints, 0.0).items():
        assert point == pytest.approx(closed[name])
