"""Native SDK choreography contract: no MCP/agent route or real hardware."""

from __future__ import annotations

from dataclasses import replace

import pytest

from soarm101_motion.agent_control import AgentAuthorityStore
from soarm101_motion.broker import AgentMotionRates
from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.broker_watchdog import BrokerWatchdog
from soarm101_motion.choreography import (
    ChoreographyExecutionError,
    ChoreographyPolicy,
    JointChoreography,
    JointFrame,
    JointTarget,
    execute_trusted_joint_target,
    execute_trusted_choreography,
    preflight_joint_target,
    preflight_choreography,
)
from soarm101_motion.config import SOARM101Config
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.exceptions import SafetyViolationError


@pytest.fixture
def supervised(tmp_path):
    executor = SDKAgentExecutor(
        config=SOARM101Config(robot_id="so101"),
        rates=AgentMotionRates(),
        simulation=True,
        simulation_actions=True,
        authority_store=AgentAuthorityStore(tmp_path / "lease.json"),
    )
    executor.execute("state", {})
    watchdog = BrokerWatchdog(
        timeout_s=8.0, validate=executor.validate_supervision,
        trip=lambda _reason: executor.execute("stop", {}),
    )
    executor.motion_guard = watchdog.check
    watchdog.start()
    watchdog.heartbeat()
    try:
        yield executor, watchdog
    finally:
        watchdog.close()
        executor.close()


def author(arm, *, delta=0.0, duration=0.5):
    base = arm.get_joint_positions().positions
    origin = tuple(base[name] for name in ARM_JOINTS)
    other = list(origin)
    other[ARM_JOINTS.index("wrist_roll")] += delta
    return JointChoreography.author(
        "approved_segment", arm.config.robot_id, "simulation",
        (JointFrame(0.0, origin), JointFrame(duration, tuple(other))),
    )


def policy(arm, segment, **overrides):
    base = dict(robot_id=arm.config.robot_id, calibration_id="simulation",
                allowed_revisions=(segment.revision,),
                max_duration_s=5.0, max_speed_deg_s=8.0,
                max_acceleration_deg_s2=25.0)
    return ChoreographyPolicy(**(base | overrides))


def target_and_policy(arm, *, delta=0.02):
    current = arm.get_joint_positions().positions
    target = [current[name] for name in ARM_JOINTS]
    target[ARM_JOINTS.index("wrist_roll")] += delta
    asset = JointTarget.author("neutral", arm.config.robot_id, "simulation", tuple(target))
    cfg = ChoreographyPolicy(
        robot_id=arm.config.robot_id, calibration_id="simulation",
        allowed_revisions=(asset.revision,), max_duration_s=2.0,
        max_speed_deg_s=8.0, max_acceleration_deg_s2=25.0,
    )
    return asset, cfg


def test_native_choreography_integrity_and_constraints(supervised):
    executor, _ = supervised
    arm = executor._session
    segment = author(arm, delta=0.02, duration=0.6)
    assert segment.revision.startswith("sha256:")
    assert segment.content_revision() == segment.revision
    assert len(segment.frames[0].joints_rad) == len(ARM_JOINTS)
    assert not hasattr(segment, "personality")
    with pytest.raises(ValueError, match="pinned"):
        replace(segment, frames=(segment.frames[0],
                                 JointFrame(0.6, tuple(0 for _ in ARM_JOINTS))))
    with pytest.raises(ValueError, match="strictly"):
        JointChoreography.author(
            "not_ordered", "so101", "simulation",
            (JointFrame(0, segment.frames[0].joints_rad),
             JointFrame(0, segment.frames[0].joints_rad)))
    with pytest.raises(ValueError, match="five finite"):
        JointFrame(0, (1.0, 2.0))
    with pytest.raises(ValueError):
        JointFrame(0, (0.0, 0.0, float("nan"), 0.0, 0.0))
    with pytest.raises(ValueError, match="30s"):
        JointChoreography.author(
            "too_long", "so101", "simulation",
            (segment.frames[0], JointFrame(31, segment.frames[1].joints_rad)))


def test_preflight_does_not_move_or_enable_torque(supervised):
    executor, _ = supervised
    arm = executor._session
    segment = author(arm, delta=0.02, duration=1.0)
    cfg = policy(arm, segment)
    commands = len(arm.backend.command_history)
    assert not arm.get_state().torque_enabled
    report = preflight_choreography(arm, segment, cfg)
    assert report.revision == segment.revision
    assert report.report()["status"] == "preflight_only_not_motion_authorization"
    assert report.sample_count >= 2
    assert len(arm.backend.command_history) == commands
    assert not arm.get_state().torque_enabled
    assert report.trajectory.metadata["source_calibration_id"] == "simulation"
    assert all(v == report.gripper_position for v in report.trajectory.gripper)


def test_pinned_target_preflight_uses_measured_start_without_motion(supervised):
    executor, _ = supervised
    arm = executor._session
    target, cfg = target_and_policy(arm)
    arm.backend._positions["shoulder_pan"] += 0.025
    before = len(arm.backend.command_history)
    report = preflight_joint_target(arm, target, cfg)
    assert report.revision == target.revision
    assert report.duration_s == pytest.approx(cfg.max_duration_s - 1.0)
    assert report.duration_s < cfg.max_duration_s
    assert report.trajectory.metadata["joint_target_revision"] == target.revision
    assert report.start_error_rad == pytest.approx(0.0)
    assert len(arm.backend.command_history) == before
    assert not arm.get_state().torque_enabled


def test_simulated_execution_reuses_sdk_motion_and_retains_human_policy(supervised, monkeypatch):
    executor, watchdog = supervised
    arm = executor._session
    segment = author(arm, delta=0.02, duration=1.0)
    cfg = policy(arm, segment)
    prior = len(arm.backend.command_history)
    # Expressive motion must NEVER write even a constant target to the gripper.
    # Fail at the authoritative backend to catch any indirect SDK motor write.
    tool_writes = []
    def forbidden_gripper_write(*args, **kwargs):
        tool_writes.append((args, kwargs))
        raise AssertionError("choreography must not command the gripper")
    monkeypatch.setattr(arm.backend, "write_tool_position", forbidden_gripper_write)
    result = execute_trusted_choreography(executor, watchdog, cfg, segment)
    assert tool_writes == []
    assert result["mode"] == "simulation"
    assert result["revision"] == segment.revision
    assert result["status"] == "sdk_execution_completed_physical_settling_not_certified"
    assert len(arm.backend.command_history) > prior
    assert arm.get_joint_positions().positions["wrist_roll"] == pytest.approx(
        segment.frames[-1].joints_rad[ARM_JOINTS.index("wrist_roll")], abs=0.005
    )
    assert executor.connection_count == 1
    # Untrusted agent route retains its independent ±2° API, and no new route
    # lets it submit a trusted choreography.
    with pytest.raises(ValueError, match="unknown"):
        executor.execute("choreography", {"revision": segment.revision})


def test_simulated_target_execution_is_pinned_and_never_writes_gripper(supervised, monkeypatch):
    executor, watchdog = supervised
    arm = executor._session
    target, cfg = target_and_policy(arm)
    arm.backend._positions["shoulder_pan"] += 0.025
    writes = []
    monkeypatch.setattr(arm.backend, "write_tool_position",
                        lambda *args, **kwargs: writes.append((args, kwargs)))
    result = execute_trusted_joint_target(executor, watchdog, cfg, target)
    assert result["revision"] == target.revision
    assert result["preflight"]["duration_s"] == pytest.approx(cfg.max_duration_s - 1.0)
    assert result["preflight"]["duration_s"] < cfg.max_duration_s
    assert result["mode"] == "simulation"
    assert writes == []
    assert arm.get_joint_positions().positions["wrist_roll"] == pytest.approx(
        target.joints_rad[ARM_JOINTS.index("wrist_roll")], abs=0.01,
    )


def test_joint_target_fails_closed_when_deadline_cannot_reserve_margin(supervised):
    executor, _ = supervised
    target, cfg = target_and_policy(executor._session)
    with pytest.raises(PermissionError, match="completion margin"):
        preflight_joint_target(
            executor._session, target, replace(cfg, max_duration_s=1.0),
        )


def test_target_completion_must_satisfy_next_choreography_entry_contract(supervised, monkeypatch):
    executor, watchdog = supervised
    arm = executor._session
    target, cfg = target_and_policy(arm)
    play = arm.play_trajectory
    def loose_completion(*args, **kwargs):
        result = play(*args, **kwargs)
        final = dict(result.final_positions)
        final[ARM_JOINTS[0]] = target.joints_rad[0] + cfg.max_start_error_rad + .001
        return replace(result, final_positions=final)
    monkeypatch.setattr(arm, "play_trajectory", loose_completion)
    with pytest.raises(RuntimeError, match="reviewed choreography entry tolerance"):
        execute_trusted_joint_target(executor, watchdog, cfg, target)
    assert watchdog.status["state"] == "stopped"
    with pytest.raises(PermissionError, match="terminal"):
        execute_trusted_joint_target(executor, watchdog, cfg, target)


def test_joint_target_does_not_retime_infeasible_velocity(supervised):
    executor, _ = supervised
    arm = executor._session
    target, cfg = target_and_policy(arm, delta=0.5)
    # The one-second authored segment is intentionally too fast for the
    # reviewed 8 deg/s ceiling; preflight must reject instead of stretching it.
    with pytest.raises(SafetyViolationError, match="speed"):
        preflight_joint_target(arm, target, cfg)


def test_native_target_timing_uses_selected_rates_instead_of_deadline(supervised):
    executor, _ = supervised
    arm = executor._session
    target, cfg = target_and_policy(arm, delta=.5)
    slow = preflight_joint_target(arm, target, replace(cfg, max_duration_s=30, target_timing="native"))
    medium = preflight_joint_target(arm, target, replace(cfg, max_duration_s=30,
        target_timing="native", max_speed_deg_s=50, max_acceleration_deg_s2=500))
    assert 0 < medium.duration_s < slow.duration_s < 29
    assert medium.duration_s + arm.config.motion_completion_timeout_s <= 30
    assert not arm.get_state().torque_enabled


def test_target_requires_exact_revision_and_existing_supervision(supervised):
    executor, watchdog = supervised
    arm = executor._session
    target, cfg = target_and_policy(arm)
    wrong = JointTarget.author("neutral", target.robot_id, target.calibration_id,
                              tuple(x + 0.01 for x in target.joints_rad))
    with pytest.raises(PermissionError, match="pinned"):
        preflight_joint_target(arm, wrong, cfg)
    original = executor.motion_guard
    executor.motion_guard = None
    with pytest.raises(PermissionError, match="watchdog"):
        execute_trusted_joint_target(executor, watchdog, cfg, target)
    executor.motion_guard = original


def test_target_execution_failure_uses_existing_terminal_stop_fence(supervised, monkeypatch):
    executor, watchdog = supervised
    arm = executor._session
    target, cfg = target_and_policy(arm)

    def fail_on_play(*_args, **_kwargs):
        raise RuntimeError("injected target execution failure")

    monkeypatch.setattr(arm, "play_trajectory", fail_on_play)
    with pytest.raises(ChoreographyExecutionError, match="injected target execution"):
        execute_trusted_joint_target(executor, watchdog, cfg, target)
    assert arm.backend.stop_count >= 1
    assert watchdog.status["state"] == "stopped"
    with pytest.raises(PermissionError):
        execute_trusted_joint_target(executor, watchdog, cfg, target)


def test_choreography_must_be_exactly_pinned_and_start_fresh(supervised):
    executor, _ = supervised
    arm = executor._session
    segment = author(arm)
    with pytest.raises(PermissionError, match="pinned"):
        preflight_choreography(arm, segment, policy(
            arm, segment, allowed_revisions=("sha256:" + "0" * 64,)
        ))
    arm.backend._positions["wrist_roll"] += 0.1
    with pytest.raises(PermissionError, match="measured start"):
        preflight_choreography(arm, segment, policy(arm, segment))


def test_no_watchdog_or_rate_override_or_unqualified_physical(supervised):
    executor, watchdog = supervised
    arm = executor._session
    segment = author(arm)
    cfg = policy(arm, segment)
    original = executor.motion_guard
    executor.motion_guard = None
    with pytest.raises(PermissionError, match="watchdog"):
        execute_trusted_choreography(executor, watchdog, cfg, segment)
    executor.motion_guard = original
    with pytest.raises(PermissionError, match="widen"):
        execute_trusted_choreography(
            executor, watchdog, policy(arm, segment, max_speed_deg_s=9), segment,
        )
    # A caller cannot toggle mode or use a nonphysical simulator for a
    # physical trial: physical state comes from SDK executor construction.
    executor.simulation_actions = False
    with pytest.raises(PermissionError, match="not operator-qualified"):
        execute_trusted_choreography(executor, watchdog, cfg, segment)


def test_attended_physical_route_requires_existing_human_lease(supervised):
    executor, watchdog = supervised
    arm = executor._session
    segment = author(arm)
    cfg = replace(policy(arm, segment), allow_physical_trial=True)
    # Reclassify this mock-backed session as the physical authority path. The
    # absent human lease must reject before preflight, torque, or SDK motion.
    executor.simulation_actions = False
    executor.physical_actions = True
    before = len(arm.backend.command_history)
    with pytest.raises(PermissionError, match="authority"):
        execute_trusted_choreography(executor, watchdog, cfg, segment)
    assert len(arm.backend.command_history) == before
    assert not arm.get_state().torque_enabled


def test_preflight_rejects_wrong_calibration_or_excess_duration(supervised):
    executor, _ = supervised
    arm = executor._session
    segment = author(arm, duration=1.0)
    wrong = ChoreographyPolicy(
        robot_id="so101", calibration_id="other",
        allowed_revisions=(segment.revision,),
    )
    with pytest.raises(PermissionError):
        preflight_choreography(arm, segment, wrong)
    with pytest.raises(PermissionError, match="duration"):
        preflight_choreography(arm, segment, policy(
            arm, segment, max_duration_s=0.2,
        ))


def test_unconfirmed_native_execution_attempt_uses_existing_stop_fence(
    supervised, monkeypatch,
):
    executor, watchdog = supervised
    arm = executor._session
    segment = author(arm, duration=0.6)
    cfg = policy(arm, segment)
    def fail_on_play(*_args, **_kwargs):
        raise RuntimeError("injected native executor failure")
    monkeypatch.setattr(arm, "play_trajectory", fail_on_play)
    with pytest.raises(ChoreographyExecutionError, match="injected native"):
        execute_trusted_choreography(executor, watchdog, cfg, segment)
    assert arm.backend.stop_count >= 1
    assert watchdog.status["state"] == "stopped"
    # STOP is terminal for the supervised session; no automatic retries.
    with pytest.raises(PermissionError):
        execute_trusted_choreography(executor, watchdog, cfg, segment)


def rest_at_stop(arm, elbow):
    """The folded sleep pose, with the elbow resting on its stop."""
    for name, value in zip(ARM_JOINTS, (0.0015, -1.70, elbow, 0.8562, 0.0046), strict=True):
        arm.backend._positions[name] = value


def test_an_arm_resting_just_past_a_joint_limit_can_still_move_back_inside(supervised):
    """A limp arm settles on its stop a hair beyond the software limit; that is a valid start."""
    executor, watchdog = supervised
    arm = executor._session
    lower, upper = arm.motion._effective_limits()["elbow_flex"]
    rest_at_stop(arm, upper + 0.004)
    # The target is the pose a hair inside the limit.
    inside = dict(arm.get_joint_positions().positions)
    inside["elbow_flex"] = upper - 0.01
    origin = JointTarget.author("rest_inside", arm.config.robot_id, "simulation",
                                tuple(inside[name] for name in ARM_JOINTS))
    cfg = replace(target_and_policy(arm)[1], allowed_revisions=(origin.revision,))
    report = preflight_joint_target(arm, origin, cfg)
    elbow = [row[ARM_JOINTS.index("elbow_flex")] for row in report.trajectory.joints_rad]
    assert elbow[0] == pytest.approx(upper + 0.004)
    assert max(elbow) <= upper + 0.004 + 1e-9  # never further out than the start
    result = execute_trusted_joint_target(executor, watchdog, cfg, origin)
    assert result["mode"] == "simulation"


def test_a_target_beyond_the_limit_is_still_refused(supervised):
    executor, _ = supervised
    arm = executor._session
    lower, upper = arm.motion._effective_limits()["elbow_flex"]
    rest_at_stop(arm, upper + 0.004)
    beyond = dict(arm.get_joint_positions().positions)
    beyond["elbow_flex"] = upper + 0.05
    target = JointTarget.author("too_far", arm.config.robot_id, "simulation",
                                tuple(beyond[name] for name in ARM_JOINTS))
    cfg = replace(target_and_policy(arm)[1], allowed_revisions=(target.revision,))
    with pytest.raises((SafetyViolationError, PermissionError)):
        preflight_joint_target(arm, target, cfg)
