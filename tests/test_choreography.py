"""Native SDK choreography contract: no MCP/agent route or real hardware."""

from __future__ import annotations

from dataclasses import replace

import pytest

from soarm101_motion.agent_control import AgentAuthorityStore
from soarm101_motion.broker import AgentMotionRates
from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.broker_watchdog import BrokerWatchdog
from soarm101_motion.choreography import (
    ChoreographyPolicy,
    JointChoreography,
    JointFrame,
    execute_trusted_choreography,
    preflight_choreography,
)
from soarm101_motion.config import SOARM101Config
from soarm101_motion.constants import ARM_JOINTS


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
