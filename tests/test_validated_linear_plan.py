from __future__ import annotations

import numpy as np
import pytest

from soarm101_motion import Pose, SOARM101
from soarm101_motion.exceptions import InvalidCommandError, StaleCartesianPlanError


def _small_inward_target(arm: SOARM101) -> Pose:
    current = arm.get_position()
    return Pose(current.position + np.array([-0.005, 0.0, 0.0]), current.rotation)


def test_guarded_linear_move_plans_only_once(monkeypatch: pytest.MonkeyPatch) -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        calls = 0
        original = arm.motion.plan_linear

        def counted(*args, **kwargs):
            nonlocal calls
            calls += 1
            return original(*args, **kwargs)

        monkeypatch.setattr(arm.motion, "plan_linear", counted)
        arm.move_linear(
            _small_inward_target(arm),
            orientation_mode="position_only",
            speed=0.01,
        )
        assert calls == 1


def test_guarded_linear_move_forwards_target_seed_through_managed_controller(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        target = _small_inward_target(arm)
        endpoint_seed = dict(arm.get_joint_positions().positions)
        calls: list[dict[str, float] | None] = []
        original = arm.motion.plan_linear

        def counted(*args, **kwargs):
            seed = kwargs.get("target_seed")
            calls.append(None if seed is None else dict(seed))
            return original(*args, **kwargs)

        monkeypatch.setattr(arm.motion, "plan_linear", counted)
        arm.move_linear(
            target,
            orientation_mode="position_only",
            speed=0.01,
            target_seed=endpoint_seed,
        )

    assert calls == [endpoint_seed]


def test_encoder_scale_noise_keeps_validated_plan(monkeypatch: pytest.MonkeyPatch) -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        target = _small_inward_target(arm)
        plan = arm.motion.plan_linear(
            target,
            tcp=arm.active_tcp,
            orientation_mode="position_only",
            speed=0.01,
        )
        original_read = arm.backend.read_joint_positions
        original_execute = arm.motion._execute_plan
        executed = []

        def noisy_read():
            positions = dict(original_read())
            positions["shoulder_pan"] += 2.0 * np.pi / 4095.0
            return positions

        def record_execute(candidate, *args, **kwargs):
            executed.append(candidate)
            return original_execute(candidate, *args, **kwargs)

        monkeypatch.setattr(arm.backend, "read_joint_positions", noisy_read)
        monkeypatch.setattr(arm.motion, "_execute_plan", record_execute)
        arm.motion.move_linear(
            target,
            tcp=arm.active_tcp,
            orientation_mode="position_only",
            speed=0.01,
        )
        assert len(executed) == 1
        assert executed[0] is plan


def test_meaningful_drift_rejects_stale_validated_plan(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        target = _small_inward_target(arm)
        arm.motion.plan_linear(
            target,
            tcp=arm.active_tcp,
            orientation_mode="position_only",
            speed=0.01,
        )
        original_read = arm.backend.read_joint_positions

        def drifted_read():
            positions = dict(original_read())
            positions["shoulder_pan"] += 0.02
            return positions

        monkeypatch.setattr(arm.backend, "read_joint_positions", drifted_read)
        with pytest.raises(InvalidCommandError, match="changed after Cartesian path validation"):
            arm.motion.move_linear(
                target,
                tcp=arm.active_tcp,
                orientation_mode="position_only",
                speed=0.01,
            )
        assert not arm.motion.is_moving


def test_facade_replans_and_revalidates_after_transient_encoder_drift(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        target = _small_inward_target(arm)
        counts = {"plans": 0, "workspace": 0, "attempts": 0}
        original_plan = arm.motion.plan_linear
        original_move = arm.motion.move_linear
        from soarm101_motion import arm as arm_module

        original_validate = arm_module.validate_workspace_path

        def counted_plan(*args, **kwargs):
            counts["plans"] += 1
            return original_plan(*args, **kwargs)

        def counted_validate(*args, **kwargs):
            counts["workspace"] += 1
            return original_validate(*args, **kwargs)

        def one_stale_start(*args, **kwargs):
            counts["attempts"] += 1
            if counts["attempts"] == 1:
                raise StaleCartesianPlanError("robot joints changed after Cartesian path validation")
            return original_move(*args, **kwargs)

        monkeypatch.setattr(arm.motion, "plan_linear", counted_plan)
        monkeypatch.setattr(arm.motion, "move_linear", one_stale_start)
        monkeypatch.setattr(arm_module, "validate_workspace_path", counted_validate)
        result = arm.move_linear(target, orientation_mode="position_only", speed=0.01)

        assert result.completed
        assert counts == {"plans": 2, "workspace": 2, "attempts": 2}


def test_facade_never_executes_persistently_stale_cartesian_plan(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        target = _small_inward_target(arm)
        counts = {"plans": 0, "workspace": 0, "attempts": 0}
        original_plan = arm.motion.plan_linear
        from soarm101_motion import arm as arm_module

        original_validate = arm_module.validate_workspace_path

        def counted_plan(*args, **kwargs):
            counts["plans"] += 1
            return original_plan(*args, **kwargs)

        def counted_validate(*args, **kwargs):
            counts["workspace"] += 1
            return original_validate(*args, **kwargs)

        def reject_stale(*args, **kwargs):
            counts["attempts"] += 1
            raise StaleCartesianPlanError("robot joints changed after Cartesian path validation")

        monkeypatch.setattr(arm.motion, "plan_linear", counted_plan)
        monkeypatch.setattr(arm.motion, "move_linear", reject_stale)
        monkeypatch.setattr(arm_module, "validate_workspace_path", counted_validate)

        with pytest.raises(StaleCartesianPlanError, match="changed after Cartesian"):
            arm.move_linear(target, orientation_mode="position_only", speed=0.01)
        assert counts == {"plans": 3, "workspace": 3, "attempts": 3}
        assert not arm.motion.is_moving


def test_facade_does_not_retry_other_motion_failures(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        target = _small_inward_target(arm)
        calls = 0

        def reject_invalid(*args, **kwargs):
            nonlocal calls
            calls += 1
            raise InvalidCommandError("unrelated motion safety rejection")

        monkeypatch.setattr(arm.motion, "move_linear", reject_invalid)
        with pytest.raises(InvalidCommandError, match="unrelated motion safety"):
            arm.move_linear(target, orientation_mode="position_only", speed=0.01)
        assert calls == 1


def test_facade_revalidates_target_only_after_stale_start(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        target = _small_inward_target(arm)
        checks = []
        from soarm101_motion import arm as arm_module

        original_check = arm_module.validate_workspace_configuration
        original_move = arm.motion.move_linear
        count = 0

        def counted_check(*args, **kwargs):
            checks.append(1)
            return original_check(*args, **kwargs)

        def transient(*args, **kwargs):
            nonlocal count
            count += 1
            if count == 1:
                raise StaleCartesianPlanError("robot joints changed after Cartesian path validation")
            return original_move(*args, **kwargs)

        monkeypatch.setattr(arm_module, "validate_workspace_configuration", counted_check)
        monkeypatch.setattr(arm.motion, "move_linear", transient)
        result = arm.move_linear(
            target,
            orientation_mode="position_only",
            speed=0.01,
            workspace_check="target_only",
        )
        assert result.completed
        assert len(checks) == 2
