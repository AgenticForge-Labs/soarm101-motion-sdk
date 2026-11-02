"""Profiles are execution choices, not dependencies of saved robot artifacts."""

import numpy as np
import pytest

from soarm101_motion import JOINT_PROFILE_MODES, SOARM101, SOARM101Config
from soarm101_motion.poses import PoseLibrary, SavedPose
from soarm101_motion.sequences import MotionSequence, SequenceLibrary, SequenceRunner, SequenceStep
from soarm101_motion.trajectories import Trajectory, TrajectoryLibrary


def test_existing_program_and_recording_survive_profile_changes(tmp_path):
    poses = PoseLibrary("sim", path=tmp_path / "poses.json")
    poses.save("target", SavedPose(
        joints={"shoulder_pan": 0.15, "shoulder_lift": 0.0, "elbow_flex": 0.0,
                "wrist_flex": 0.0, "wrist_roll": 0.0},
        gripper=0.5, tcp_xyz_rpy=(0, 0, 0, 0, 0, 0), source="simulation",
    ))
    trajectory = Trajectory(
        timestamps_s=np.array([0.0, 0.5, 1.0]),
        joints_rad=np.array([[0, 0, 0, 0, 0], [0.04, 0, 0, 0, 0], [0.08, 0, 0, 0, 0]]),
        gripper=np.array([1.0, 0.8, 0.6]), metadata={"source": "simulation"},
    )
    trajectories = TrajectoryLibrary("sim", root=tmp_path / "trajectories")
    entry = trajectories.save("recording", trajectory, kind="raw")
    sequences = SequenceLibrary("sim", path=tmp_path / "sequences.json")
    sequences.save(MotionSequence("program", steps=(
        SequenceStep("point", {"name": "target"}),
        SequenceStep("trajectory", {"name": "recording", "kind": "raw"}),
    )))
    original_files = {path: path.read_bytes() for path in (
        poses.path, sequences.path, entry.data_path, entry.metadata_path,
    )}
    recording_samples = None
    for profile in JOINT_PROFILE_MODES:
        cfg = SOARM101Config(joint_profile_mode=profile, enable_workspace_checks=False)
        with SOARM101.simulated(realtime=False, config=cfg) as arm:
            arm.enable()
            selected = []
            original_planner = arm.motion._plan_joint_motion

            def capture_profile(*args, **kwargs):
                selected.append(arm.motion._resolve_joint_profile_mode(kwargs.get("profile_mode")))
                return original_planner(*args, **kwargs)

            arm.motion._plan_joint_motion = capture_profile
            loaded = trajectories.load("recording", kind="raw")
            planned = arm.motion.plan_recorded_trajectory(loaded, move_to_start=True)
            actual_samples = (planned.command_samples, planned.gripper_samples, planned.duration_s)
            if recording_samples is None:
                recording_samples = actual_samples
            else:
                assert actual_samples == recording_samples
            runner = SequenceRunner(arm, pose_library=poses, trajectory_library=trajectories)
            assert runner.run(sequences.require("program")).completed
            assert selected and all(mode == profile for mode in selected)
            assert arm.get_joint_positions().positions["shoulder_pan"] == pytest.approx(0.08)
            assert arm.tool.get_position() == pytest.approx(0.6)
    assert all(path.read_bytes() == contents for path, contents in original_files.items())
