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

    assert calls
    assert all(
        speed == TELEOP_SERVO_SPEED_RAW
        and acceleration == TELEOP_SERVO_ACCELERATION_RAW
        for speed, acceleration in calls
    )

