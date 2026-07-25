import numpy as np
import pytest

from soarm101_motion import Pose, SOARM101
from soarm101_motion.exceptions import InvalidCommandError, SafetyViolationError


def test_simulated_joint_move_and_gripper() -> None:
    arm = SOARM101.simulated()
    with arm:
        arm.enable()
        result = arm.move_joints([0.2, -0.4, 0.6, 0.1, 0.0])
        assert result.completed
        positions = arm.get_joint_positions().positions
        assert positions["elbow_flex"] == pytest.approx(0.6)
        assert len(arm.backend.command_history) > 2  # type: ignore[attr-defined]
        arm.tool.close()
        assert arm.tool.is_closed
        arm.tool.open()
        assert arm.tool.is_open


def test_motion_requires_torque() -> None:
    with SOARM101.simulated() as arm:
        with pytest.raises(InvalidCommandError):
            arm.move_joints([0.1, 0, 0, 0, 0])


def test_linear_move_runs_common_ik_path() -> None:
    arm = SOARM101.simulated()
    with arm:
        arm.enable()
        arm.move_joints([0.25, -0.45, 0.65, 0.20, -0.15])
        start = arm.get_position()
        target = Pose(start.position + np.array([0.012, 0.0, 0.008]), start.rotation)
        result = arm.move_linear(target, orientation_mode="position_only", speed=0.02)
        assert result.completed
        end = arm.get_position()
        assert np.linalg.norm(end.position - target.position) < 0.003


def test_xarm_joint_alias_degrees() -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        arm.set_servo_angle([0, -10, 20, 0, 5], is_radian=False, speed=30, mvacc=60)
        values = arm.get_servo_angle(is_radian=False)
        assert values == pytest.approx([0, -10, 20, 0, 5], abs=0.05)


def test_xarm_cartesian_alias_uses_millimeters() -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        current = arm.get_position()
        arm.set_position(x=5, relative=True, orientation_mode="position_only", speed=20)
        moved = arm.get_position()
        assert moved.position[0] - current.position[0] == pytest.approx(0.005, abs=0.002)


def test_out_of_range_joint_rejected() -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        with pytest.raises(SafetyViolationError):
            arm.move_joints([10.0, 0, 0, 0, 0])
