from __future__ import annotations

import pytest

from soarm101_motion import Pose, SOARM101
from soarm101_motion.exceptions import InvalidCommandError, SafetyViolationError



def test_calibrated_extensions_follow_measured_range_with_one_degree_margin() -> None:
    from types import SimpleNamespace

    import numpy as np

    from soarm101_motion.constants import ARM_JOINTS

    measured_deg = {
        "shoulder_pan": (-121.14285714285717, 121.14285714285717),
        "shoulder_lift": (-105.05494505494505, 105.05494505494505),
        "elbow_flex": (-96.96703296703296, 96.96703296703296),
        "wrist_flex": (-103.91208791208791, 103.91208791208791),
        "wrist_roll": (-168.79120879120882, 168.79120879120882),
    }

    with SOARM101.simulated() as arm:
        arm.backend.calibration = SimpleNamespace(
            motors={
                name: SimpleNamespace(
                    radians_limits=tuple(np.deg2rad(measured_deg[name]))
                )
                for name in ARM_JOINTS
            }
        )
        limits = arm.get_joint_limits()
        controller_limits = arm.motion._effective_limits()

    for name in ARM_JOINTS:
        assert limits[name] == pytest.approx(controller_limits[name])

    assert np.degrees(limits["shoulder_pan"]) == pytest.approx(
        (measured_deg["shoulder_pan"][0] + 1.0, measured_deg["shoulder_pan"][1] - 1.0)
    )
    assert np.degrees(limits["shoulder_lift"]) == pytest.approx(
        (
            measured_deg["shoulder_lift"][0] + 1.0,
            measured_deg["shoulder_lift"][1] - 1.0,
        )
    )
    assert np.degrees(limits["elbow_flex"]) == pytest.approx(
        (measured_deg["elbow_flex"][0] + 1.0, measured_deg["elbow_flex"][1] - 1.0)
    )
    assert np.degrees(limits["wrist_flex"]) == pytest.approx(
        (measured_deg["wrist_flex"][0] + 1.0, measured_deg["wrist_flex"][1] - 1.0)
    )
    assert np.degrees(limits["wrist_roll"]) == pytest.approx(
        (measured_deg["wrist_roll"][0] + 1.0, measured_deg["wrist_roll"][1] - 1.0)
    )


def test_final_target_joint_execution_writes_endpoint_once() -> None:
    from soarm101_motion.constants import ARM_JOINTS

    with SOARM101.simulated() as arm:
        arm.enable()
        start = dict(arm.get_joint_positions().positions)
        target = dict(start)
        target["shoulder_pan"] += 0.20
        target["elbow_flex"] -= 0.10

        result = arm.move_joints(
            target,
            speed=0.2,
            acceleration=0.5,
            execution_mode="final_target",
            workspace_check="off",
        )

        history = list(arm.backend.command_history)  # type: ignore[attr-defined]

    assert result.completed is True
    assert len(history) == 1
    assert history[0] == pytest.approx({name: target[name] for name in ARM_JOINTS})


def test_final_target_uses_synchronized_per_joint_servo_speeds() -> None:
    from types import SimpleNamespace

    from soarm101_motion.constants import ARM_JOINTS

    class Motor:
        radians_limits = (-2.0, 2.0)

        @staticmethod
        def radians_to_raw(value: float) -> int:
            return int(round(2000.0 + value * 1000.0))

    with SOARM101.simulated() as arm:
        arm.backend.calibration = SimpleNamespace(
            motors={name: Motor() for name in ARM_JOINTS}
        )
        arm.enable()
        target = dict(arm.get_joint_positions().positions)
        target["shoulder_pan"] += 0.20
        target["elbow_flex"] -= 0.10
        captured: dict[str, object] = {}
        original = arm.backend.write_joint_positions

        def recorded(positions, *, speed_raw=None, acceleration_raw=None):
            captured["speed_raw"] = speed_raw
            captured["acceleration_raw"] = acceleration_raw
            return original(
                positions,
                speed_raw=speed_raw,
                acceleration_raw=acceleration_raw,
            )

        arm.backend.write_joint_positions = recorded  # type: ignore[method-assign]
        arm.move_joints(
            target,
            speed=0.2,
            acceleration=0.5,
            execution_mode="final_target",
            workspace_check="off",
        )

    speeds = captured["speed_raw"]
    assert isinstance(speeds, dict)
    assert set(speeds) == set(ARM_JOINTS)
    assert all(int(value) >= 1 for value in speeds.values())
    assert int(speeds["shoulder_pan"]) > int(speeds["elbow_flex"])


def test_final_target_monitor_rejects_reverse_motion() -> None:
    from soarm101_motion.constants import ARM_JOINTS

    with SOARM101.simulated() as arm:
        arm.enable()
        start = dict(arm.get_joint_positions().positions)
        target = dict(start)
        target["shoulder_pan"] += 0.50
        previous = dict(start)
        arm.backend._positions["shoulder_pan"] -= 0.10  # type: ignore[attr-defined]

        with pytest.raises(SafetyViolationError, match="opposite the final target"):
            arm.motion._monitor_final_target_motion(  # type: ignore[attr-defined]
                start,
                target,
                previous,
            )


def test_streamed_joint_execution_remains_default() -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        target = dict(arm.get_joint_positions().positions)
        target["shoulder_pan"] += 0.20

        arm.move_joints(
            target,
            speed=0.2,
            acceleration=0.5,
            workspace_check="off",
        )
        history = list(arm.backend.command_history)  # type: ignore[attr-defined]

    assert len(history) > 1
    assert history[-1]["shoulder_pan"] == pytest.approx(target["shoulder_pan"])


def test_invalid_joint_execution_mode_is_rejected() -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        target = dict(arm.get_joint_positions().positions)
        target["shoulder_pan"] += 0.05
        with pytest.raises(InvalidCommandError, match="execution_mode"):
            arm.move_joints(
                target,
                execution_mode="burst",  # type: ignore[arg-type]
                workspace_check="off",
            )


def test_saved_pose_can_use_final_target_execution() -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        target = dict(arm.get_joint_positions().positions)
        target["wrist_roll"] += 0.10
        result = arm.move_joints_from_saved_pose(
            target,
            speed=0.2,
            acceleration=0.5,
            execution_mode="final_target",
        )
        history = list(arm.backend.command_history)  # type: ignore[attr-defined]

    assert result.completed is True
    assert len(history) == 1
    assert history[-1]["wrist_roll"] == pytest.approx(target["wrist_roll"])


def test_joint_planner_preserves_exact_validated_endpoint_at_effective_limit() -> None:
    from types import SimpleNamespace

    import numpy as np

    from soarm101_motion.constants import ARM_JOINTS

    initial_positions = {name: 0.0 for name in ARM_JOINTS}
    initial_positions["elbow_flex"] = -1.6

    with SOARM101.simulated(initial_positions=initial_positions) as arm:
        arm.backend.calibration = SimpleNamespace(
            motors={
                name: SimpleNamespace(radians_limits=(-2.0, 2.0))
                for name in ARM_JOINTS
            }
        )
        arm.backend.calibration.motors["elbow_flex"] = SimpleNamespace(
            radians_limits=tuple(
                np.deg2rad((-96.96703296703296, 96.96703296703296))
            )
        )
        target = arm.get_joint_limits()["elbow_flex"][1]

        # This is the hardware regression: mathematically evaluating the endpoint
        # as start + (target - start) can round one ULP above target.
        reconstructed = initial_positions["elbow_flex"] + (
            target - initial_positions["elbow_flex"]
        )
        assert reconstructed > target

        arm.enable()
        result = arm.move_joints(
            {"elbow_flex": target},
            speed=0.2,
            acceleration=0.5,
            workspace_check="off",
        )
        final = arm.backend.command_history[-1]["elbow_flex"]  # type: ignore[attr-defined]

    assert result.completed is True
    assert final == target


def test_builtin_sleep_pose_is_calibration_relative_and_guarded() -> None:
    from types import SimpleNamespace

    import numpy as np

    from soarm101_motion.constants import ARM_JOINTS, STOCK_GRIPPER

    measured_deg = {
        "shoulder_pan": (-121.14285714285717, 121.14285714285717),
        "shoulder_lift": (-105.05494505494505, 105.05494505494505),
        "elbow_flex": (-96.96703296703296, 96.96703296703296),
        "wrist_flex": (-103.91208791208791, 103.91208791208791),
        "wrist_roll": (-168.79120879120882, 168.79120879120882),
    }

    with SOARM101.simulated() as arm:
        motors = {
            name: SimpleNamespace(
                radians_limits=tuple(np.deg2rad(measured_deg[name]))
            )
            for name in ARM_JOINTS
        }
        motors[STOCK_GRIPPER] = SimpleNamespace(
            radians_limits=(0.0, float(np.deg2rad(100.0)))
        )
        arm.backend.calibration = SimpleNamespace(motors=motors)
        sleep = arm.get_sleep_joint_positions()
        sleep_gripper = arm.get_sleep_gripper_position()
        arm.enable()
        with pytest.raises(SafetyViolationError, match="coarse self-clearance"):
            arm.move_joints(sleep, speed=0.2, acceleration=0.5)
        result = arm.move_sleep(speed=0.2, acceleration=0.5)
        final = dict(arm.get_joint_positions().positions)
        final_gripper = arm.tool.get_position()

    assert np.degrees(sleep["shoulder_pan"]) == pytest.approx(0.0)
    assert np.degrees(sleep["shoulder_lift"]) == pytest.approx(
        measured_deg["shoulder_lift"][0] + 1.0
    )
    assert np.degrees(sleep["elbow_flex"]) == pytest.approx(
        measured_deg["elbow_flex"][1] - 1.0
    )
    assert np.degrees(sleep["wrist_flex"]) == pytest.approx(
        measured_deg["wrist_flex"][0] + 1.0
    )
    assert np.degrees(sleep["wrist_roll"]) == pytest.approx(0.0)
    assert sleep_gripper == pytest.approx(0.01)
    assert final_gripper == pytest.approx(sleep_gripper)
    assert result.final_positions[STOCK_GRIPPER] == pytest.approx(sleep_gripper)
    assert result.completed is True
    for name, target in sleep.items():
        assert final[name] == pytest.approx(
            target,
            abs=arm.config.joint_position_tolerance_rad,
        )



def test_saved_pose_can_monotonically_exit_existing_self_clearance(monkeypatch) -> None:
    from soarm101_motion.constants import ARM_JOINTS

    import soarm101_motion.safety as safety_module

    with SOARM101.simulated() as arm:
        arm.enable()
        arm.move_sleep(speed=0.2, acceleration=0.5)
        clearances = iter((0.019, 0.020, 0.022, 0.026))

        monkeypatch.setattr(
            safety_module,
            "minimum_workspace_self_clearance",
            lambda *args, **kwargs: next(clearances, 0.026),
        )
        monkeypatch.setattr(
            safety_module,
            "validate_workspace_configuration",
            lambda *args, **kwargs: None,
        )
        monkeypatch.setattr(
            safety_module,
            "validate_workspace_path",
            lambda *args, **kwargs: None,
        )
        target = {name: 0.0 for name in ARM_JOINTS}
        result = arm.move_joints_from_saved_pose(
            target,
            speed=0.2,
            acceleration=0.5,
        )

    assert result.completed is True


def test_saved_pose_rejects_path_that_moves_deeper_into_self_clearance(monkeypatch) -> None:
    from soarm101_motion.constants import ARM_JOINTS

    import soarm101_motion.safety as safety_module

    with SOARM101.simulated() as arm:
        arm.enable()
        arm.move_sleep(speed=0.2, acceleration=0.5)
        clearances = iter((0.019, 0.018, 0.026))
        monkeypatch.setattr(
            safety_module,
            "minimum_workspace_self_clearance",
            lambda *args, **kwargs: next(clearances, 0.026),
        )
        monkeypatch.setattr(
            safety_module,
            "validate_workspace_configuration",
            lambda *args, **kwargs: None,
        )
        target = {name: 0.0 for name in ARM_JOINTS}
        with pytest.raises(SafetyViolationError, match="moves deeper"):
            arm.move_joints_from_saved_pose(
                target,
                speed=0.2,
                acceleration=0.5,
            )


def test_saved_pose_can_remain_inside_nonworsening_self_clearance(monkeypatch) -> None:
    from soarm101_motion.constants import ARM_JOINTS

    import soarm101_motion.safety as safety_module

    with SOARM101.simulated() as arm:
        arm.enable()
        arm.move_sleep(speed=0.2, acceleration=0.5)
        monkeypatch.setattr(
            safety_module,
            "minimum_workspace_self_clearance",
            lambda *args, **kwargs: 0.020,
        )
        monkeypatch.setattr(
            safety_module,
            "validate_workspace_configuration",
            lambda *args, **kwargs: None,
        )
        target = {name: 0.0 for name in ARM_JOINTS}
        result = arm.move_joints_from_saved_pose(
            target,
            speed=0.2,
            acceleration=0.5,
        )

    assert result.completed is True

def test_public_ik_uses_executable_calibrated_joint_limits(monkeypatch) -> None:
    from types import SimpleNamespace

    import numpy as np

    from soarm101_motion.constants import ARM_JOINTS
    from soarm101_motion.types import IKResult

    with SOARM101.simulated() as arm:
        arm.backend.calibration = SimpleNamespace(
            motors={
                name: SimpleNamespace(radians_limits=(-2.2, 2.2))
                for name in ARM_JOINTS
            }
        )
        observed = {}

        def fake_solve(target, *, seed, tcp, options):
            del target, seed, tcp
            observed["limits"] = options.joint_limits
            return IKResult(
                success=True,
                joints={name: 0.0 for name in ARM_JOINTS},
                position_error_m=0.0,
                orientation_error_rad=0.0,
                iterations=1,
                message="ok",
            )

        monkeypatch.setattr(arm.ik, "solve", fake_solve)
        expected_limits = arm.get_joint_limits()
        arm.solve_ik(
            Pose(np.array([0.1, 0.0, 0.1]), np.eye(3)),
            orientation_mode="position_only",
        )

    assert observed["limits"] is not None
    for name in ARM_JOINTS:
        assert observed["limits"][name] == pytest.approx(expected_limits[name])



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
            return SimpleNamespace(
                success=True,
                joints=joints,
                position_error_m=0.0,
                message="synthetic command-rate IK solution",
            )

        monkeypatch.setattr(arm.motion.ik, "solve", nonlinear_solution)
        monkeypatch.setattr(
            arm.motion,
            "_smooth_position_only_cartesian_samples",
            lambda samples, cartesian, **kwargs: samples,
        )
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


def test_linear_plan_retries_failed_intermediate_ik_with_multi_start(monkeypatch) -> None:
    from types import SimpleNamespace

    import numpy as np

    with SOARM101.simulated() as arm:
        arm.enable()
        original = arm.motion.ik.solve
        calls: list[bool] = []
        failed_once = False

        def flaky(target, *, seed, tcp, options):
            nonlocal failed_once
            calls.append(bool(options.multi_start))
            if not options.multi_start and not failed_once:
                failed_once = True
                return SimpleNamespace(
                    success=False,
                    joints=dict(seed),
                    position_error_m=0.000556,
                    message="synthetic single-start miss",
                )
            return original(target, seed=seed, tcp=tcp, options=options)

        monkeypatch.setattr(arm.motion.ik, "solve", flaky)
        current = arm.get_position()
        target = Pose(
            current.position + np.array([-0.010, 0.0, 0.004]),
            current.rotation,
        )
        plan = arm.motion.plan_linear(
            target,
            orientation_mode="position_only",
            speed=0.01,
            acceleration=0.05,
        )

    assert plan.command_samples
    assert failed_once is True
    assert any(calls[index : index + 2] == [False, True] for index in range(len(calls) - 1))


def test_position_only_linear_plan_uses_endpoint_seed_for_reverse_ik_fallback(
    monkeypatch,
) -> None:
    from types import SimpleNamespace

    import numpy as np

    with SOARM101.simulated() as arm:
        arm.enable()
        start = dict(arm.get_joint_positions().positions)
        start_pose = arm.get_position()
        target = Pose(
            start_pose.position + np.array([-0.012, 0.0, 0.0]),
            start_pose.rotation,
        )
        start_x = float(start_pose.position[0])
        target_seed = dict(start)
        target_seed["wrist_roll"] += 0.04
        induced_failures = 0

        def branch_sensitive_solution(pose, *, seed, tcp, options):
            nonlocal induced_failures
            del tcp, options
            progress = (float(pose.position[0]) - start_x) / -0.012
            reverse_branch = float(seed["wrist_roll"]) > float(start["wrist_roll"]) + 1e-8
            if not reverse_branch and 0.45 <= progress <= 0.55:
                induced_failures += 1
                return SimpleNamespace(
                    success=False,
                    joints=dict(seed),
                    position_error_m=0.001,
                    message="forward continuation trapped in local IK minimum",
                )

            joints = dict(start)
            joints["shoulder_pan"] += 0.02 * progress
            if reverse_branch:
                joints["wrist_roll"] += 0.04 * progress
            return SimpleNamespace(
                success=True,
                joints=joints,
                position_error_m=0.0,
                message="synthetic branch solution",
            )

        monkeypatch.setattr(arm.motion.ik, "solve", branch_sensitive_solution)
        monkeypatch.setattr(
            arm.motion,
            "_smooth_position_only_cartesian_samples",
            lambda samples, cartesian, **kwargs: samples,
        )
        plan = arm.motion.plan_linear(
            target,
            orientation_mode="position_only",
            speed=0.01,
            acceleration=0.05,
            target_seed=target_seed,
        )

    assert induced_failures >= 1
    assert plan.command_samples
    assert plan.command_samples[0] == start
    assert plan.command_samples[-1]["wrist_roll"] == pytest.approx(
        start["wrist_roll"] + 0.04
    )


def test_plan_linear_from_is_read_only_and_does_not_require_torque() -> None:
    import numpy as np

    with SOARM101.simulated() as arm:
        start = dict(arm.get_joint_positions().positions)
        start_pose = arm.model.forward(start, tcp=arm.active_tcp)
        target = Pose(
            start_pose.position + np.array([-0.005, 0.0, 0.0]),
            start_pose.rotation,
        )

        plan = arm.motion.plan_linear_from(
            start,
            target,
            tcp=arm.active_tcp,
            orientation_mode="position_only",
            speed=0.01,
            acceleration=0.05,
        )

        assert plan.command_samples[0] == pytest.approx(start)
        assert arm.backend.get_hardware_state().torque_enabled is False



def test_plan_linear_from_accepts_calibration_bounded_diagnostic_limits(monkeypatch) -> None:
    from types import SimpleNamespace

    import numpy as np

    from soarm101_motion.constants import ARM_JOINTS, JOINT_LIMITS

    class FakeMotor:
        # Wider than every nominal model joint and the +0.10 rad wrist-flex
        # diagnostic extension used below.
        radians_limits = (-3.2, 3.2)

    with SOARM101.simulated() as arm:
        arm.backend.calibration = SimpleNamespace(
            motors={name: FakeMotor() for name in ARM_JOINTS}
        )
        start = dict(arm.get_joint_positions().positions)
        start_pose = arm.model.forward(start, tcp=arm.active_tcp)
        target = Pose(
            start_pose.position + np.array([-0.005, 0.0, 0.0]),
            start_pose.rotation,
        )
        diagnostic_limits = dict(JOINT_LIMITS)
        diagnostic_limits["wrist_flex"] = (
            JOINT_LIMITS["wrist_flex"][0],
            JOINT_LIMITS["wrist_flex"][1] + 0.10,
        )
        observed = []
        original = arm.motion.ik.solve

        def record_limits(target_pose, *, seed, tcp, options):
            observed.append(options.joint_limits)
            return original(target_pose, seed=seed, tcp=tcp, options=options)

        monkeypatch.setattr(arm.motion.ik, "solve", record_limits)
        plan = arm.motion.plan_linear_from(
            start,
            target,
            tcp=arm.active_tcp,
            orientation_mode="position_only",
            speed=0.01,
            acceleration=0.05,
            limits_override=diagnostic_limits,
        )

    assert plan.command_samples
    assert observed
    assert all(item == diagnostic_limits for item in observed if item is not None)



def test_plan_linear_from_rejects_diagnostic_limits_beyond_calibration() -> None:
    from types import SimpleNamespace

    import numpy as np

    from soarm101_motion.constants import ARM_JOINTS, JOINT_LIMITS
    from soarm101_motion.exceptions import SafetyViolationError

    class FakeMotor:
        radians_limits = (-2.5, 2.5)

    with SOARM101.simulated() as arm:
        arm.backend.calibration = SimpleNamespace(
            motors={name: FakeMotor() for name in ARM_JOINTS}
        )
        start = dict(arm.get_joint_positions().positions)
        start_pose = arm.model.forward(start, tcp=arm.active_tcp)
        target = Pose(
            start_pose.position + np.array([-0.005, 0.0, 0.0]),
            start_pose.rotation,
        )
        diagnostic_limits = dict(JOINT_LIMITS)

        with pytest.raises(SafetyViolationError, match="exceeds calibrated range"):
            arm.motion.plan_linear_from(
                start,
                target,
                tcp=arm.active_tcp,
                orientation_mode="position_only",
                speed=0.01,
                acceleration=0.05,
                limits_override=diagnostic_limits,
            )


def test_plan_linear_from_uses_explicit_start_without_changing_backend_state() -> None:
    import numpy as np

    with SOARM101.simulated() as arm:
        arm.enable()
        measured = dict(arm.get_joint_positions().positions)
        start = dict(measured)
        start["shoulder_pan"] += 0.02
        start_pose = arm.model.forward(start, tcp=arm.active_tcp)
        target = Pose(
            start_pose.position + np.array([-0.005, 0.0, 0.0]),
            start_pose.rotation,
        )

        plan = arm.motion.plan_linear_from(
            start,
            target,
            tcp=arm.active_tcp,
            orientation_mode="position_only",
            speed=0.01,
            acceleration=0.05,
        )

        assert plan.command_samples[0] == pytest.approx(start)
        assert dict(arm.get_joint_positions().positions) == pytest.approx(measured)


def test_cartesian_ik_failure_reports_sample_residual_and_conditioning(
    monkeypatch,
) -> None:
    from types import SimpleNamespace

    import numpy as np

    from soarm101_motion.exceptions import IKError

    with SOARM101.simulated() as arm:
        arm.enable()
        start = dict(arm.get_joint_positions().positions)
        start_pose = arm.model.forward(start, tcp=arm.active_tcp)
        target = Pose(
            start_pose.position + np.array([-0.010, 0.0, 0.0]),
            start_pose.rotation,
        )

        def fail(target_pose, *, seed, tcp, options):
            del target_pose, tcp, options
            return SimpleNamespace(
                success=False,
                joints=dict(seed),
                position_error_m=0.00065,
                message="forced diagnostic failure",
            )

        monkeypatch.setattr(arm.motion.ik, "solve", fail)
        with pytest.raises(IKError) as excinfo:
            arm.motion.plan_linear_from(
                start,
                target,
                tcp=arm.active_tcp,
                orientation_mode="position_only",
                speed=0.01,
                acceleration=0.05,
            )

    message = str(excinfo.value)
    assert "sample" in message
    assert "line progress=" in message
    assert "residual=(" in message
    assert "sigma_min=" in message
    assert "nearest effective joint limit=" in message


def test_cosine_cruise_profile_uses_requested_speed_as_cruise_ceiling() -> None:
    from soarm101_motion.motion.controller import MotionController

    duration = MotionController._cosine_cruise_duration(0.220, 0.020, 0.100)

    assert duration == pytest.approx(11.3141592654, rel=1e-6)
    assert MotionController._cosine_cruise_progress(
        0.220,
        0.020,
        0.100,
        duration / 2.0,
    ) == pytest.approx(0.5, abs=1e-8)


def test_position_only_cartesian_reprojection_reduces_joint_jerk(monkeypatch) -> None:
    from types import SimpleNamespace

    with SOARM101.simulated() as arm:
        arm.enable()
        start = dict(arm.get_joint_positions().positions)
        samples = []
        poses = []
        for index in range(7):
            sample = dict(start)
            sample["shoulder_pan"] += 0.01 * index
            sample["wrist_roll"] += [0.0, 0.03, -0.02, 0.04, -0.01, 0.02, 0.0][index]
            samples.append(sample)
            poses.append(arm.get_position())

        original_roughness = arm.motion._joint_path_roughness(tuple(samples))

        def return_seed(target, *, seed, tcp, options):
            del target, tcp, options
            return SimpleNamespace(joints=dict(seed))

        monkeypatch.setattr(arm.motion.ik, "solve_or_raise", return_seed)
        refined = arm.motion._smooth_position_only_cartesian_samples(
            tuple(samples),
            tuple(poses),
            tcp=None,
            limits=arm.motion._effective_limits(),
        )

    assert arm.motion._joint_path_roughness(refined) < original_roughness
    assert refined[0] == samples[0]
    assert refined[-1] == samples[-1]


def test_position_only_linear_timing_ignores_target_rotation() -> None:
    import numpy as np

    with SOARM101.simulated() as arm:
        arm.enable()
        current = arm.get_position()
        position = current.position + np.array([-0.010, 0.0, 0.0])
        rotated = np.diag([-1.0, -1.0, 1.0])

        unchanged = arm.motion.plan_linear(
            Pose(position, current.rotation),
            orientation_mode="position_only",
            speed=0.020,
            acceleration=0.100,
        )
        arbitrary_rotation = arm.motion.plan_linear(
            Pose(position, rotated),
            orientation_mode="position_only",
            speed=0.020,
            acceleration=0.100,
        )

    assert arbitrary_rotation.duration_s == pytest.approx(unchanged.duration_s)
    assert len(arbitrary_rotation.command_samples) == len(unchanged.command_samples)


def test_cartesian_execution_uses_teleop_servo_profile_even_with_calibration() -> None:
    from types import SimpleNamespace

    from soarm101_motion.constants import (
        TELEOP_SERVO_ACCELERATION_RAW,
        TELEOP_SERVO_SPEED_RAW,
    )

    class FakeMotor:
        radians_limits = (-3.0, 3.0)

        @staticmethod
        def radians_to_raw(value):
            return int(round(2048 + float(value) * 1000.0))

    with SOARM101.simulated() as arm:
        arm.enable()
        arm.backend.calibration = SimpleNamespace(
            motors={
                name: FakeMotor()
                for name in (
                    "shoulder_pan",
                    "shoulder_lift",
                    "elbow_flex",
                    "wrist_flex",
                    "wrist_roll",
                )
            }
        )
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
        target = Pose(current.position + [-0.010, 0.0, 0.005], current.rotation)
        arm.move_linear(
            target,
            orientation_mode="position_only",
            speed=0.020,
            acceleration=0.100,
        )

    assert calls
    assert all(
        speed == TELEOP_SERVO_SPEED_RAW
        and acceleration == TELEOP_SERVO_ACCELERATION_RAW
        for speed, acceleration in calls
    )


def test_calibrated_motion_skips_redundant_quantized_encoder_targets() -> None:
    from types import SimpleNamespace

    from soarm101_motion.constants import ARM_JOINTS

    class FakeMotor:
        radians_limits = (-3.0, 3.0)

        @staticmethod
        def radians_to_raw(value):
            return int(round(2048 + float(value) * 100.0))

    with SOARM101.simulated() as arm:
        arm.backend.calibration = SimpleNamespace(
            motors={name: FakeMotor() for name in ARM_JOINTS}
        )
        arm.enable()
        start = dict(arm.get_joint_positions().positions)
        target = dict(start)
        target["shoulder_pan"] += 0.08
        plan = arm.motion._plan_joint_motion(
            start,
            target,
            speed=0.05,
            acceleration=0.20,
            limits=arm.motion._effective_limits(),
        )
        planned_keys = [
            arm.motion._encoder_target_key(sample)
            for sample in plan.command_samples
        ]
        assert any(
            previous == current
            for previous, current in zip(planned_keys, planned_keys[1:], strict=False)
        )

        calls = []
        original = arm.backend.write_joint_positions

        def recorded(positions, *, speed_raw=None, acceleration_raw=None):
            calls.append(dict(positions))
            return original(
                positions,
                speed_raw=speed_raw,
                acceleration_raw=acceleration_raw,
            )

        arm.backend.write_joint_positions = recorded  # type: ignore[method-assign]
        result = arm.move_joints(
            target,
            speed=0.05,
            acceleration=0.20,
            workspace_check="off",
        )

        call_keys = [arm.motion._encoder_target_key(sample) for sample in calls]

    assert result.completed is True
    assert len(calls) < len(plan.command_samples) - 1
    assert calls[-1] == pytest.approx(target)
    assert all(
        previous != current
        for previous, current in zip(call_keys[:-1], call_keys[1:-1], strict=False)
    )


def test_joint_move_can_use_per_joint_synchronized_servo_speeds() -> None:
    from types import SimpleNamespace

    class FakeMotor:
        radians_limits = (-3.0, 3.0)

        @staticmethod
        def radians_to_raw(value):
            return int(round(2048 + float(value) * 1000.0))

    with SOARM101.simulated() as arm:
        arm.enable()
        arm.backend.calibration = SimpleNamespace(
            motors={
                name: FakeMotor()
                for name in (
                    "shoulder_pan",
                    "shoulder_lift",
                    "elbow_flex",
                    "wrist_flex",
                    "wrist_roll",
                )
            }
        )
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
        target = dict(arm.get_joint_positions().positions)
        target["shoulder_pan"] += 0.08
        target["shoulder_lift"] += 0.04
        arm.move_joints(
            target,
            speed=0.20,
            acceleration=0.60,
            servo_acceleration_raw=254,
            synchronize_servo_arrival=True,
        )

    mapped = [speed for speed, _ in calls if isinstance(speed, dict)]
    assert mapped
    assert all(
        set(speed)
        == {
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
        }
        for speed in mapped
    )
    assert all(all(1 <= value <= 3400 for value in speed.values()) for speed in mapped)


def test_joint_move_rejects_fixed_and_synchronized_servo_speed_together() -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        target = dict(arm.get_joint_positions().positions)
        target["shoulder_pan"] += 0.02
        with pytest.raises(InvalidCommandError, match="cannot be combined"):
            arm.move_joints(
                target,
                servo_speed_raw=0,
                synchronize_servo_arrival=True,
            )


def test_joint_target_only_workspace_mode_skips_full_path_guard(monkeypatch) -> None:
    import soarm101_motion.arm as arm_module

    with SOARM101.simulated() as arm:
        arm.enable()
        target = dict(arm.get_joint_positions().positions)
        target["shoulder_pan"] += 0.02
        calls = {"target": 0}

        def reject_full_path(*args, **kwargs):
            del args, kwargs
            raise AssertionError("full joint workspace guard should not run")

        def accept_target(*args, **kwargs):
            del args, kwargs
            calls["target"] += 1

        monkeypatch.setattr(arm_module, "validate_workspace_path", reject_full_path)
        monkeypatch.setattr(
            arm_module,
            "validate_workspace_configuration",
            accept_target,
        )

        arm.move_joints(target, workspace_check="target_only")

    assert calls["target"] == 1


def test_invalid_joint_workspace_mode_is_rejected() -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        target = dict(arm.get_joint_positions().positions)
        target["shoulder_pan"] += 0.02
        with pytest.raises(InvalidCommandError, match="workspace_check"):
            arm.move_joints(
                target,
                workspace_check="anything",  # type: ignore[arg-type]
            )
