from __future__ import annotations

import pytest

from soarm101_motion import Pose, SOARM101
from soarm101_motion.exceptions import InvalidCommandError


def test_guarded_linear_move_plans_only_once() -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        calls = 0
        original = arm.motion.plan_linear

        def counted(*args, **kwargs):
            nonlocal calls
            calls += 1
            return original(*args, **kwargs)

        arm.motion.plan_linear = counted  # type: ignore[method-assign]
        current = arm.get_position()
        target = Pose(current.position + [-0.005, 0.0, 0.005], current.rotation)
        arm.move_linear(target, orientation_mode="position_only", speed=0.01)
        assert calls == 1


def test_target_only_workspace_mode_skips_full_path_guard(monkeypatch) -> None:
    import soarm101_motion.arm as arm_module

    with SOARM101.simulated() as arm:
        arm.enable()
        current = arm.get_position()
        target = Pose(current.position + [-0.005, 0.0, 0.005], current.rotation)
        calls = {"target": 0}

        def reject_full_path(*args, **kwargs):
            del args, kwargs
            raise AssertionError("full path workspace guard should not run")

        def accept_target(*args, **kwargs):
            del args, kwargs
            calls["target"] += 1

        monkeypatch.setattr(arm_module, "validate_workspace_path", reject_full_path)
        monkeypatch.setattr(arm_module, "validate_workspace_configuration", accept_target)

        arm.move_linear(
            target,
            orientation_mode="position_only",
            speed=0.01,
            workspace_check="target_only",
        )
        assert calls["target"] == 1


def test_invalid_linear_workspace_mode_is_rejected() -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        current = arm.get_position()
        with pytest.raises(InvalidCommandError, match="workspace_check"):
            arm.move_linear(
                current,
                workspace_check="anything",  # type: ignore[arg-type]
            )

def test_move_linear_uses_responsive_servo_profile() -> None:
    from soarm101_motion.constants import (
        TELEOP_SERVO_ACCELERATION_RAW,
        TELEOP_SERVO_SPEED_RAW,
    )

    with SOARM101.simulated() as arm:
        arm.enable()
        calls = []
        original = arm.backend.write_joint_positions

        def recorded(positions, *, speed_raw=None, acceleration_raw=None):
            calls.append((speed_raw, acceleration_raw))
            return original(
                positions,
                speed_raw=speed_raw,
                acceleration_raw=acceleration_raw,
            )

        arm.backend.write_joint_positions = recorded  # type: ignore[method-assign]
        current = arm.get_position()
        target = Pose(current.position + [-0.005, 0.0, 0.005], current.rotation)
        arm.move_linear(target, orientation_mode="position_only", speed=0.01)
        arm.backend.write_joint_positions = original  # type: ignore[method-assign]

    assert calls
    assert all(
        speed == TELEOP_SERVO_SPEED_RAW
        and acceleration == TELEOP_SERVO_ACCELERATION_RAW
        for speed, acceleration in calls
    )



def test_linear_plan_solves_ik_at_each_command_rate_cartesian_sample(monkeypatch) -> None:
    from types import SimpleNamespace

    import numpy as np

    with SOARM101.simulated() as arm:
        arm.enable()
        start_joints = dict(arm.get_joint_positions().positions)
        start_pose = arm.get_position()
        target = Pose(start_pose.position + np.array([-0.012, 0.0, 0.0]), start_pose.rotation)
        start_x = float(start_pose.position[0])

        def nonlinear_solution(pose, *, seed, tcp, options):
            del seed, tcp, options
            progress_like = (float(pose.position[0]) - start_x) / -0.012
            joints = dict(start_joints)
            joints["shoulder_pan"] += 0.02 * progress_like**2
            joints["wrist_roll"] += 0.01 * progress_like**3
            return SimpleNamespace(joints=joints)

        monkeypatch.setattr(arm.motion.ik, "solve_or_raise", nonlinear_solution)
        plan = arm.motion.plan_linear(
            target,
            orientation_mode="position_only",
            speed=0.01,
            acceleration=0.05,
        )

    assert len(plan.command_samples) == len(plan.cartesian_waypoints)
    assert len(plan.command_samples) > 10
    for command, pose in zip(
        plan.command_samples[1:],
        plan.cartesian_waypoints[1:],
        strict=True,
    ):
        progress_like = (float(pose.position[0]) - start_x) / -0.012
        assert command["shoulder_pan"] == pytest.approx(
            start_joints["shoulder_pan"] + 0.02 * progress_like**2,
            abs=1e-10,
        )
        assert command["wrist_roll"] == pytest.approx(
            start_joints["wrist_roll"] + 0.01 * progress_like**3,
            abs=1e-10,
        )
