from pathlib import Path

import numpy as np
import pytest

from soarm101_motion import SOARM101, SOARM101Config
from soarm101_motion.exceptions import InvalidCommandError, SafetyViolationError
from soarm101_motion.hardware import SimulationBackend
from soarm101_motion.poses import PoseLibrary, SavedPose
from soarm101_motion.primitives import MotionPrimitive, MotionPrimitiveLibrary
from soarm101_motion.sequences import MotionSequence, SequenceLibrary, SequenceRunner, SequenceStep
from soarm101_motion.trajectories import Trajectory, TrajectoryLibrary


def test_guarded_joint_stream_in_simulation() -> None:
    config = SOARM101Config(
        enable_workspace_checks=False,
        effort_safety_enabled=False,
        command_frequency_hz=50.0,
        max_joint_speed=2.0,
        max_joint_acceleration=50.0,
    )
    backend = SimulationBackend(realtime=False)
    arm = SOARM101(config, backend=backend)
    arm.connect()
    arm.enable()
    try:
        arm.start_joint_stream()
        assert arm.motion.is_streaming
        result = arm.stream_joint_target(
            {
                "shoulder_pan": 0.01,
                "shoulder_lift": 0.0,
                "elbow_flex": 0.0,
                "wrist_flex": 0.0,
                "wrist_roll": 0.0,
            },
            gripper=0.6,
        )
        assert result.accepted
        assert not result.completed
        assert arm.tool.get_position() == pytest.approx(0.6)

        arm.stream_joint_target(
            {
                "shoulder_pan": 0.02,
                "shoulder_lift": 0.0,
                "elbow_flex": 0.0,
                "wrist_flex": 0.0,
                "wrist_roll": 0.0,
            }
        )
        with pytest.raises(InvalidCommandError, match="streaming"):
            arm.move_joints([0, 0, 0, 0, 0])
        arm.stop_joint_stream()
        assert not arm.motion.is_streaming
    finally:
        arm.disconnect()


def test_stream_rejects_large_command_step() -> None:
    config = SOARM101Config(
        enable_workspace_checks=False,
        effort_safety_enabled=False,
        max_joint_speed=10.0,
        max_joint_acceleration=100.0,
    )
    arm = SOARM101(config, backend=SimulationBackend(realtime=False))
    arm.connect()
    arm.enable()
    try:
        arm.start_joint_stream()
        with pytest.raises(SafetyViolationError, match="command step"):
            arm.stream_joint_target(
                {
                    "shoulder_pan": 0.5,
                    "shoulder_lift": 0.0,
                    "elbow_flex": 0.0,
                    "wrist_flex": 0.0,
                    "wrist_roll": 0.0,
                }
            )
        arm.stop_joint_stream()
    finally:
        arm.disconnect()


def _pose(joint: float, gripper: float = 1.0) -> SavedPose:
    return SavedPose(
        joints={
            "shoulder_pan": joint,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
        },
        gripper=gripper,
        tcp_xyz_rpy=(0, 0, 0, 0, 0, 0),
    )


def test_sequence_library_and_runner(tmp_path: Path) -> None:
    poses = PoseLibrary("test", path=tmp_path / "poses.json")
    poses.save("home", _pose(0.0))
    poses.save("rest", _pose(0.0))
    poses.save("point_a", _pose(0.05))

    trajectories = TrajectoryLibrary("test", root=tmp_path / "trajectories")
    primitives = MotionPrimitiveLibrary("test", path=tmp_path / "primitives.json")
    sequences = SequenceLibrary("test", path=tmp_path / "sequences.json")

    sequence = MotionSequence(
        "demo",
        (
            SequenceStep("point", {"name": "point_a", "mode": "joint"}),
            SequenceStep("gripper", {"position": 0.25}),
            SequenceStep("wait", {"seconds": 0.0}),
            SequenceStep("home", {}),
        ),
    )
    sequences.save(sequence)
    assert sequences.require("demo").steps[0].kind == "point"

    config = SOARM101Config(enable_workspace_checks=False, effort_safety_enabled=False)
    arm = SOARM101(config, backend=SimulationBackend(realtime=False))
    arm.connect()
    arm.enable()
    try:
        runner = SequenceRunner(
            arm,
            pose_library=poses,
            trajectory_library=trajectories,
            primitive_library=primitives,
        )
        progress: list[tuple[int, str]] = []
        result = runner.run(
            sequence,
            on_progress=lambda index, _total, _step, status: progress.append(
                (index, status)
            ),
        )
        assert result.completed
        assert arm.get_joint_positions().positions["shoulder_pan"] == pytest.approx(0.0)
        assert arm.tool.get_position() == pytest.approx(0.25)
        assert (0, "started") in progress
        assert (3, "completed") in progress
    finally:
        arm.disconnect()


def test_motion_primitive_library(tmp_path: Path) -> None:
    library = MotionPrimitiveLibrary("test", path=tmp_path / "primitives.json")
    primitive = MotionPrimitive(
        name="wave_gentle",
        trajectory_name="wave_trim_v2",
        tags=("greeting", "happy"),
        loopable=False,
        interruptible=True,
        default_speed_scale=0.8,
    )
    library.save(primitive)
    loaded = MotionPrimitiveLibrary("test", path=tmp_path / "primitives.json")
    assert loaded.require("wave_gentle") == primitive


def _trajectory() -> Trajectory:
    times = np.linspace(0.0, 1.0, 11)
    joints = np.zeros((11, 5))
    joints[:, 0] = np.sin(times * np.pi) * 0.1
    gripper = np.linspace(1.0, 0.0, 11)
    return Trajectory(times, joints, gripper)


def test_advanced_trajectory_edits_are_non_destructive() -> None:
    original = _trajectory()
    smooth = original.smooth(5)
    assert smooth.sample_count == original.sample_count
    assert smooth.joints_rad[0] == pytest.approx(original.joints_rad[0])
    assert smooth.joints_rad[-1] == pytest.approx(original.joints_rad[-1])

    held = original.insert_hold(0.5, 0.4)
    assert held.duration_s == pytest.approx(1.4)

    keyframed = original.set_keyframe(0.55, "shoulder_pan", 0.123)
    index = int(np.where(np.isclose(keyframed.timestamps_s, 0.55))[0][0])
    assert keyframed.joints_rad[index, 0] == pytest.approx(0.123)

    marked = original.add_marker(0.4, "beat", marker_type="show")
    assert marked.metadata["markers"][0]["label"] == "beat"

    deleted = original.delete_region(0.3, 0.7)
    assert deleted.duration_s < original.duration_s

    repeated = original.repeat(3)
    assert repeated.duration_s > original.duration_s * 2.9

    assert original.duration_s == pytest.approx(1.0)
    assert "markers" not in original.metadata
