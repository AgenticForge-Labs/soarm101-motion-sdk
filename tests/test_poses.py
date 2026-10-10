import math
from pathlib import Path

import pytest

from soarm101_motion import SOARM101
from soarm101_motion.poses import (
    HOME_POSE_NAME,
    REST_POSE_NAME,
    PoseLibrary,
    SavedPose,
    sleep_joint_positions,
    sleep_up_joint_positions,
)


def test_saved_pose_round_trip(tmp_path: Path) -> None:
    arm = SOARM101.simulated()
    arm.connect()
    try:
        pose = SavedPose.capture(arm)
    finally:
        arm.disconnect()

    path = tmp_path / "poses.json"
    library = PoseLibrary("test-arm", path=path)
    library.save(HOME_POSE_NAME, pose)
    library.save(REST_POSE_NAME, pose)

    loaded = PoseLibrary("test-arm", path=path)
    assert loaded.names() == ("home", "rest")
    assert loaded.require("home").joints == pose.joints
    assert loaded.require("home").gripper == pytest.approx(pose.gripper)
    assert loaded.require("home").tcp_xyz_rpy == pytest.approx(pose.tcp_xyz_rpy)
    assert loaded.require("home").source_robot_id == pose.source_robot_id
    assert loaded.require("home").source_calibration_id == pose.source_calibration_id



def test_saved_pose_capture_uses_one_measured_joint_snapshot(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    arm = SOARM101.simulated()
    arm.connect()
    original_read = arm.backend.read_joint_positions
    reads = 0

    def counted_read():
        nonlocal reads
        reads += 1
        return original_read()

    monkeypatch.setattr(arm.backend, "read_joint_positions", counted_read)
    try:
        pose = SavedPose.capture(arm)
        expected_tcp = arm.model.forward(
            pose.joints,
            tcp=arm.active_tcp,
        ).xyz_rpy()
    finally:
        arm.disconnect()

    assert reads == 1
    assert pose.tcp_xyz_rpy == pytest.approx(expected_tcp)


def test_saved_pose_round_trip_preserves_explicit_target_binding(tmp_path: Path) -> None:
    pose = SavedPose(
        joints={
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
        },
        gripper=0.5,
        tcp_xyz_rpy=(0, 0, 0, 0, 0, 0),
        source="leader",
        source_robot_id="leader",
        source_calibration_id="sha256:leader",
        target_robot_id="follower",
        target_calibration_id="sha256:follower",
    )
    library = PoseLibrary("follower", path=tmp_path / "poses.json")
    library.save("point", pose)

    loaded = PoseLibrary("follower", path=tmp_path / "poses.json").require("point")
    assert loaded.source_robot_id == "leader"
    assert loaded.source_calibration_id == "sha256:leader"
    assert loaded.target_robot_id == "follower"
    assert loaded.target_calibration_id == "sha256:follower"


def test_saved_pose_validation() -> None:
    with pytest.raises(ValueError, match="joints must match"):
        SavedPose(joints={}, gripper=0.5, tcp_xyz_rpy=(0, 0, 0, 0, 0, 0))

    with pytest.raises(ValueError, match="gripper"):
        SavedPose(
            joints={
                "shoulder_pan": 0,
                "shoulder_lift": 0,
                "elbow_flex": 0,
                "wrist_flex": 0,
                "wrist_roll": 0,
            },
            gripper=2.0,
            tcp_xyz_rpy=(0, 0, 0, 0, 0, 0),
        )


def test_sleep_joint_positions_use_smoother_calibration_relative_wrist() -> None:
    limits = {
        "shoulder_pan": (-2.0, 2.0),
        "shoulder_lift": (-1.8, 1.8),
        "elbow_flex": (-1.6, 1.6),
        "wrist_flex": (-1.7, 1.7),
        "wrist_roll": (-2.8, 3.0),
    }

    sleep = sleep_joint_positions(limits)
    sleep_up = sleep_up_joint_positions(limits)

    assert sleep == pytest.approx(
        {
            "shoulder_pan": 0.0,
            "shoulder_lift": -1.8 + math.radians(2.0),
            "elbow_flex": 1.6 - math.radians(2.0),
            "wrist_flex": -1.7 + 0.75 * 3.4,
            "wrist_roll": 0.1,
        }
    )
    assert sleep_up == pytest.approx(
        {
            "shoulder_pan": 0.0,
            "shoulder_lift": -1.8 + math.radians(2.0),
            "elbow_flex": 1.6 - math.radians(2.0),
            "wrist_flex": -1.7 + math.radians(2.0),
            "wrist_roll": 0.1,
        }
    )


def test_sleep_family_endpoint_targets_remain_inside_effective_limits() -> None:
    # The previous implementation selected the effective limit exactly; a
    # planned endpoint could then be rejected for rounding even though the
    # advertised angle appeared to match that limit.
    limits = {
        "shoulder_pan": (-2.0, 2.0),
        "shoulder_lift": (-1.816054, 1.816054),
        "elbow_flex": (-1.674941, 1.674941),
        "wrist_flex": (-1.8, 1.8),
        "wrist_roll": (-2.9, 2.9),
    }
    for derived in (sleep_joint_positions(limits), sleep_up_joint_positions(limits)):
        assert all(limits[name][0] < angle < limits[name][1]
                   for name, angle in derived.items())
        assert derived["shoulder_lift"] == pytest.approx(
            limits["shoulder_lift"][0] + math.radians(2.0)
        )
        assert derived["elbow_flex"] == pytest.approx(
            limits["elbow_flex"][1] - math.radians(2.0)
        )


def test_sleep_inset_scales_for_unusually_narrow_calibration() -> None:
    limits = {name: (-0.01, 0.01) for name in (
        "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"
    )}
    for derived in (sleep_joint_positions(limits), sleep_up_joint_positions(limits)):
        assert all(limits[name][0] < angle < limits[name][1]
                   for name, angle in derived.items())
        assert derived["shoulder_lift"] == pytest.approx(-0.008)
        assert derived["elbow_flex"] == pytest.approx(0.008)


@pytest.mark.parametrize("close_gripper", [False, True])
def test_sleep_optional_gripper_close_preserves_arm_execution(
    monkeypatch: pytest.MonkeyPatch, close_gripper: bool,
) -> None:
    from soarm101_motion.motion import MotionHandle
    from soarm101_motion.types import MotionResult

    arm = SOARM101.simulated()
    joint_calls = []
    tool_calls = []

    def completed_handle() -> MotionHandle:
        handle = MotionHandle(
            lambda event: MotionResult(accepted=True, completed=True, final_positions={})
        )
        handle.start()
        return handle

    def move_joints(target, **kwargs):
        joint_calls.append((dict(target), dict(kwargs)))
        return completed_handle()

    def move_tool(target, *, wait=True):
        tool_calls.append((target, wait))
        return completed_handle()

    monkeypatch.setattr(arm, "get_sleep_joint_positions", lambda: {"shoulder_pan": 0.0})
    monkeypatch.setattr(arm, "get_sleep_gripper_position", lambda: 0.025)
    monkeypatch.setattr(arm, "move_joints", move_joints)
    monkeypatch.setattr(arm.tool, "move", move_tool)

    result = arm.move_sleep(close_gripper=close_gripper)
    assert result.completed
    assert len(joint_calls) == 1
    assert joint_calls[0][1]["workspace_check"] == "off"
    assert joint_calls[0][1]["wait"] is False
    assert tool_calls == ([(0.025, False)] if close_gripper else [])
