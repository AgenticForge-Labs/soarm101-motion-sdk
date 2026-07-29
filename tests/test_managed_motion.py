from __future__ import annotations

from soarm101_motion import Pose, SOARM101


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
