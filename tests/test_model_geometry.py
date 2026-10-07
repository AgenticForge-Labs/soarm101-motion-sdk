"""URDF parity and stock jaw behavior independent of the GUI projection."""

from importlib.resources import files
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from soarm101_motion.kinematics import SO101KinematicModel
from soarm101_motion.kinematics.model import STOCK_JAW_JOINT, STOCK_JAW_LIMITS
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


def test_presentation_link_segments_follow_packaged_urdf_visual_bodies():
    root = ET.fromstring(files("soarm101_motion.models").joinpath("so101.urdf").read_text())
    model = SO101KinematicModel()
    joints = np.array([0.2, -0.35, 0.5, -0.4, 0.3])
    frames = model.link_frames(joints)
    segments = model.presentation_link_segments(joints)
    links = {
        "base": "base_link",
        "shoulder": "shoulder_link",
        "upper_arm": "upper_arm_link",
        "lower_arm": "lower_arm_link",
        "wrist": "wrist_link",
        "gripper_body": "gripper_link",
    }

    for segment_name, link_name in links.items():
        link = root.find(f"link[@name='{link_name}']")
        visual = link.find("visual")
        visual_frame = _origin(visual)
        geometry = visual.find("geometry")
        box = geometry.find("box")
        cylinder = geometry.find("cylinder")
        if box is not None:
            size = np.fromstring(box.get("size"), sep=" ")
            axis = int(np.argmax(size))
            length = float(size[axis])
        else:
            axis = 2
            length = float(cylinder.get("length"))
        local_axis = np.zeros(3)
        local_axis[axis] = 1.0
        center = visual_frame[:3, 3]
        direction = visual_frame[:3, :3] @ local_axis
        expected_local = (
            center - direction * length / 2.0,
            center + direction * length / 2.0,
        )
        frame = frames[link_name]
        expected = tuple(
            frame[:3, 3] + frame[:3, :3] @ point
            for point in expected_local
        )
        actual = segments[segment_name]
        expected_thickness = (
            2.0 * float(cylinder.get("radius"))
            if cylinder is not None
            else float(np.max(np.delete(size, axis)))
        )
        assert model.presentation_link_thicknesses()[segment_name] == pytest.approx(
            expected_thickness
        )
        direct_error = np.linalg.norm(actual[0] - expected[0]) + np.linalg.norm(
            actual[1] - expected[1]
        )
        reversed_error = np.linalg.norm(actual[0] - expected[1]) + np.linalg.norm(
            actual[1] - expected[0]
        )
        assert min(direct_error, reversed_error) < 1e-8


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
