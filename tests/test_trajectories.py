from pathlib import Path

import numpy as np
import pytest

from soarm101_motion import SOARM101, SOARM101Config
from soarm101_motion.hardware import SimulationBackend
from soarm101_motion.trajectories import Trajectory, TrajectoryLibrary


def demo_trajectory() -> Trajectory:
    return Trajectory(
        timestamps_s=np.array([10.0, 10.5, 11.0]),
        joints_rad=np.array(
            [
                [0.0, 0.0, 0.0, 0.0, 0.0],
                [0.05, -0.03, 0.04, 0.01, -0.02],
                [0.10, -0.06, 0.08, 0.02, -0.04],
            ]
        ),
        gripper=np.array([1.0, 0.7, 0.4]),
        metadata={"source": "leader"},
    )


def test_trajectory_normalizes_crops_retimes_and_resamples() -> None:
    trajectory = demo_trajectory()
    assert trajectory.timestamps_s.tolist() == pytest.approx([0.0, 0.5, 1.0])
    cropped = trajectory.crop(0.25, 0.75)
    assert cropped.duration_s == pytest.approx(0.5)
    assert cropped.joints_rad[0, 0] == pytest.approx(0.025)
    assert cropped.joints_rad[-1, 0] == pytest.approx(0.075)

    slower = cropped.retime(0.5)
    assert slower.duration_s == pytest.approx(1.0)

    resampled = slower.resample(20.0)
    assert resampled.sample_count == 21
    assert resampled.joints_rad[0] == pytest.approx(slower.joints_rad[0])
    assert resampled.joints_rad[-1] == pytest.approx(slower.joints_rad[-1])


def test_trajectory_library_is_non_destructive(tmp_path: Path) -> None:
    library = TrajectoryLibrary("arm", root=tmp_path)
    trajectory = demo_trajectory()
    entry = library.save("wave_raw", trajectory, kind="raw")
    assert entry.data_path.is_file()
    assert library.load("wave_raw", kind="raw").joints_rad == pytest.approx(
        trajectory.joints_rad
    )
    with pytest.raises(FileExistsError):
        library.save("wave_raw", trajectory, kind="raw")

    edited = trajectory.crop(0.2, 0.8)
    library.save("wave_trim", edited, kind="edited")
    assert {(item.kind, item.name) for item in library.entries()} == {
        ("raw", "wave_raw"),
        ("edited", "wave_trim"),
    }


def test_recorded_trajectory_replay_in_simulation() -> None:
    config = SOARM101Config(
        enable_workspace_checks=False,
        effort_safety_enabled=False,
        max_joint_speed=2.0,
        max_joint_acceleration=10.0,
    )
    backend = SimulationBackend(realtime=False)
    arm = SOARM101(config, backend=backend)
    arm.connect()
    arm.enable()
    try:
        trajectory = demo_trajectory()
        result = arm.play_trajectory(trajectory, speed_scale=1.0)
        assert result.completed
        final = arm.get_joint_positions().positions
        assert final["shoulder_pan"] == pytest.approx(0.10)
        assert arm.tool.get_position() == pytest.approx(0.4)
    finally:
        arm.disconnect()


def test_recorded_trajectory_honors_selected_gripper_speed() -> None:
    config = SOARM101Config(
        enable_workspace_checks=False,
        effort_safety_enabled=False,
        max_joint_speed=2.0,
        max_joint_acceleration=10.0,
    )
    backend = SimulationBackend(realtime=False)
    arm = SOARM101(config, backend=backend)
    arm.connect()
    arm.enable()
    writes: list[tuple[str, float, int | None]] = []
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
        result = arm.play_trajectory(
            demo_trajectory(),
            speed_scale=1.0,
            gripper_speed_raw=700,
        )
        assert result.completed
        assert writes
        assert {speed for _actuator, _position, speed in writes} == {700}
    finally:
        arm.disconnect()
