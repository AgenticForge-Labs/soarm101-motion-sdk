from __future__ import annotations

import numpy as np
import pytest

from soarm101_motion import Pose, SOARM101
from soarm101_motion.exceptions import InvalidCommandError


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
