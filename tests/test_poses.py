from pathlib import Path

import pytest

from soarm101_motion import SOARM101
from soarm101_motion.poses import (
    HOME_POSE_NAME,
    REST_POSE_NAME,
    PoseLibrary,
    SavedPose,
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
