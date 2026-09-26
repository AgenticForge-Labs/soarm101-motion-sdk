from pathlib import Path
import time

import numpy as np
import pytest

from soarm101_motion import SOARM101, SOARM101Config
from soarm101_motion.constants import ARM_JOINTS
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


def test_stream_passes_gripper_speed_to_backend() -> None:
    config = SOARM101Config(enable_workspace_checks=False, effort_safety_enabled=False)
    backend = SimulationBackend(realtime=False)
    arm = SOARM101(config, backend=backend)
    arm.connect()
    arm.enable()
    writes = []
    original_write = backend.write_tool_position

    def capture_write(actuator, position, *, speed_raw=None, acceleration_raw=None):
        writes.append((actuator, position, speed_raw))
        original_write(
            actuator, position, speed_raw=speed_raw, acceleration_raw=acceleration_raw
        )

    backend.write_tool_position = capture_write
    try:
        arm.start_joint_stream()
        neutral = {name: 0.0 for name in ARM_JOINTS}
        arm.stream_joint_target(neutral, gripper=0.7, gripper_speed_raw=500)
        assert writes[-1] == ("so101_gripper", 0.7, 500)
        with pytest.raises(InvalidCommandError, match="gripper speed"):
            arm.stream_joint_target(neutral, gripper=0.7, gripper_speed_raw=0)
        arm.stop_joint_stream()
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


def test_stream_rate_changes_velocity_validation() -> None:
    config = SOARM101Config(
        enable_workspace_checks=False,
        effort_safety_enabled=False,
        command_frequency_hz=50.0,
        default_joint_speed=0.1,
        max_joint_speed=0.2,
        max_joint_acceleration=100.0,
    )
    arm = SOARM101(config, backend=SimulationBackend(realtime=False))
    arm.connect()
    arm.enable()
    target = {
        "shoulder_pan": 0.03,
        "shoulder_lift": 0.0,
        "elbow_flex": 0.0,
        "wrist_flex": 0.0,
        "wrist_roll": 0.0,
    }
    try:
        arm.start_joint_stream(frequency_hz=5.0)
        assert arm.stream_joint_target(target).accepted
        arm.stop_joint_stream()

        arm.start_joint_stream(frequency_hz=50.0)
        with pytest.raises(SafetyViolationError, match="streamed joint speed"):
            arm.stream_joint_target(
                {
                    **target,
                    "shoulder_pan": target["shoulder_pan"] + 0.03,
                }
            )
        arm.stop_joint_stream()
    finally:
        arm.disconnect()


def test_teleop_stream_can_use_higher_limits_than_planned_motion() -> None:
    config = SOARM101Config(
        enable_workspace_checks=False,
        effort_safety_enabled=False,
        max_joint_speed=1.0,
        max_joint_acceleration=5.0,
        teleop_max_joint_speed=1.2,
        teleop_max_joint_acceleration=6.0,
    )
    arm = SOARM101(config, backend=SimulationBackend(realtime=False))
    arm.connect()
    arm.enable()
    try:
        with pytest.raises(SafetyViolationError, match="joint speed"):
            arm.move_joints([0.01, 0, 0, 0, 0], speed=1.1)
        arm.start_joint_stream(frequency_hz=20.0)
        baseline = arm.get_joint_positions().positions
        first = dict(baseline, shoulder_pan=baseline["shoulder_pan"] + 0.045)
        second = dict(first, shoulder_pan=first["shoulder_pan"] + 0.06)
        assert arm.stream_joint_target(first).accepted
        assert arm.stream_joint_target(second).accepted
        arm.stop_joint_stream()
    finally:
        arm.disconnect()


def test_joint_stream_stop_reissues_hold_after_stream_state_was_cleared() -> None:
    config = SOARM101Config(
        enable_workspace_checks=False,
        effort_safety_enabled=False,
        command_frequency_hz=20.0,
        max_joint_speed=10.0,
        max_joint_acceleration=100.0,
    )
    backend = SimulationBackend(realtime=False)
    arm = SOARM101(config, backend=backend)
    arm.connect()
    arm.enable()
    try:
        arm.start_joint_stream(frequency_hz=10.0)
        first = {name: 0.0 for name in ARM_JOINTS}
        first["shoulder_pan"] = 0.02
        assert arm.stream_joint_target(first).accepted

        def fail_write(_positions, **_kwargs) -> None:
            raise RuntimeError("simulated transport write failure")

        backend.write_joint_positions = fail_write  # type: ignore[method-assign]
        second = dict(first, shoulder_pan=0.04)
        with pytest.raises(RuntimeError, match="transport write failure"):
            arm.stream_joint_target(second)
        assert not arm.motion.is_streaming
        calls_after_failed_sample = backend.stop_count
        assert calls_after_failed_sample == 1

        # Worker shutdown must be able to issue a second hold even though the
        # failed sample already cleared the controller's streaming state.
        arm.stop_joint_stream(hold=True)
        assert backend.stop_count == calls_after_failed_sample + 1
    finally:
        arm.disconnect()


def test_stream_rate_cannot_exceed_command_frequency_ceiling() -> None:
    config = SOARM101Config(
        enable_workspace_checks=False,
        effort_safety_enabled=False,
        command_frequency_hz=20.0,
    )
    arm = SOARM101(config, backend=SimulationBackend(realtime=False))
    arm.connect()
    arm.enable()
    try:
        with pytest.raises(SafetyViolationError, match="frequency"):
            arm.start_joint_stream(frequency_hz=50.0)
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
        # The final Home step restores Home's stored gripper state after the
        # explicit 0.25 gripper step.
        assert arm.tool.get_position() == pytest.approx(1.0)
        assert (0, "started") in progress
        assert (3, "completed") in progress
    finally:
        arm.disconnect()


def test_sequence_saved_pose_joint_override_changes_only_requested_joint(
    tmp_path: Path,
) -> None:
    poses = PoseLibrary("pattern-test", path=tmp_path / "poses.json")
    base = SavedPose(
        joints={
            "shoulder_pan": 0.10,
            "shoulder_lift": -0.20,
            "elbow_flex": 0.30,
            "wrist_flex": -0.40,
            "wrist_roll": 0.50,
        },
        gripper=0.65,
        tcp_xyz_rpy=(0, 0, 0, 0, 0, 0),
    )
    poses.save("radial_base", base)
    trajectories = TrajectoryLibrary("pattern-test", root=tmp_path / "trajectories")
    arm = SOARM101(
        SOARM101Config(enable_workspace_checks=False, effort_safety_enabled=False),
        backend=SimulationBackend(realtime=False),
    )
    arm.connect()
    arm.enable()
    try:
        runner = SequenceRunner(
            arm,
            pose_library=poses,
            trajectory_library=trajectories,
        )
        result = runner.run(
            MotionSequence(
                "pan_demo",
                (
                    SequenceStep(
                        "point",
                        {
                            "name": "radial_base",
                            "mode": "joint",
                            "joint_overrides_deg": {"shoulder_pan": 30.0},
                        },
                    ),
                ),
            )
        )
        assert result.completed
        measured = arm.get_joint_positions().positions
        assert measured["shoulder_pan"] == pytest.approx(np.deg2rad(30.0))
        for joint in ("shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"):
            assert measured[joint] == pytest.approx(base.joints[joint])

        with pytest.raises(ValueError, match="joint-overridden"):
            runner.run(
                MotionSequence(
                    "bad_linear_pan",
                    (
                        SequenceStep(
                            "point",
                            {
                                "name": "radial_base",
                                "mode": "linear",
                                "joint_overrides_deg": {"shoulder_pan": 20.0},
                            },
                        ),
                    ),
                )
            )
    finally:
        arm.disconnect()


def test_sequence_runner_honors_gripper_speed_for_gripper_steps(tmp_path: Path) -> None:
    poses = PoseLibrary("speed-test", path=tmp_path / "poses.json")
    trajectories = TrajectoryLibrary("speed-test", root=tmp_path / "trajectories")
    backend = SimulationBackend(realtime=False)
    arm = SOARM101(
        SOARM101Config(enable_workspace_checks=False, effort_safety_enabled=False),
        backend=backend,
    )
    arm.connect()
    arm.enable()
    writes = []
    original_write = backend.write_tool_position

    def capture_write(actuator, position, *, speed_raw=None, acceleration_raw=None):
        writes.append((actuator, float(position), speed_raw))
        original_write(
            actuator,
            position,
            speed_raw=speed_raw,
            acceleration_raw=acceleration_raw,
        )

    backend.write_tool_position = capture_write  # type: ignore[method-assign]
    try:
        runner = SequenceRunner(
            arm,
            pose_library=poses,
            trajectory_library=trajectories,
            gripper_speed_raw=600,
        )
        result = runner.run(
            MotionSequence("grip_speed", (SequenceStep("gripper", {"position": 0.35}),))
        )
        assert result.completed
        assert writes[-1] == ("so101_gripper", 0.35, 600)
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


def test_sequence_pause_resume_during_wait(tmp_path: Path) -> None:
    poses = PoseLibrary("pause-test", path=tmp_path / "poses.json")
    trajectories = TrajectoryLibrary("pause-test", root=tmp_path / "trajectories")
    arm = SOARM101(
        SOARM101Config(enable_workspace_checks=False, effort_safety_enabled=False),
        backend=SimulationBackend(realtime=False),
    )
    arm.connect()
    arm.enable()
    try:
        runner = SequenceRunner(
            arm,
            pose_library=poses,
            trajectory_library=trajectories,
        )
        sequence = MotionSequence(
            "pause_demo",
            (
                SequenceStep("wait", {"seconds": 0.15}),
                SequenceStep("gripper", {"position": 0.4}),
            ),
        )
        handle = runner.run(sequence, wait=False)
        time.sleep(0.03)
        runner.pause()
        assert runner.is_paused
        time.sleep(0.08)
        assert not handle.done
        runner.resume()
        result = handle.wait(1.0)
        assert result.completed
        assert arm.tool.get_position() == pytest.approx(0.4)
    finally:
        arm.disconnect()


def test_stream_accepts_calibrated_rest_but_rejects_farther_out():
    from soarm101_motion.calibration import MotorCalibration, SO101Calibration
    from soarm101_motion.constants import MOTOR_IDS
    backend = SimulationBackend(initial_positions={"shoulder_lift": -1.798265})
    backend.calibration = SO101Calibration(motors={
        name: MotorCalibration(motor_id, 0, 0, 862, 3232)
        for name, motor_id in MOTOR_IDS.items()
    })
    arm = SOARM101(SOARM101Config(), backend=backend)
    arm.connect()
    arm.enable()
    try:
        arm.start_joint_stream(frequency_hz=5)
        baseline = backend.read_joint_positions()
        arm.stream_joint_target(baseline)
        before = len(backend.command_history)
        with pytest.raises(SafetyViolationError, match="outside"):
            arm.stream_joint_target(dict(baseline, shoulder_lift=-1.80))
        assert len(backend.command_history) == before
        arm.stream_joint_target(dict(baseline, shoulder_lift=-1.79))
    finally:
        arm.disconnect()


def test_stream_cannot_extend_past_calibration_from_invalid_start():
    from soarm101_motion.calibration import MotorCalibration, SO101Calibration
    from soarm101_motion.constants import MOTOR_IDS
    backend = SimulationBackend(initial_positions={"shoulder_lift": -2.0})
    backend.calibration = SO101Calibration(motors={
        name: MotorCalibration(motor_id, 0, 0, 862, 3232)
        for name, motor_id in MOTOR_IDS.items()
    })
    arm = SOARM101(SOARM101Config(), backend=backend)
    arm.connect()
    arm.enable()
    try:
        with pytest.raises(SafetyViolationError, match="outside"):
            arm.start_joint_stream()
        assert not backend.command_history
    finally:
        arm.disconnect()
