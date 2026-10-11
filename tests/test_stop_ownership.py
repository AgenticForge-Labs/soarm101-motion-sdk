"""Hardware-free regression for one canonical STOP writer and cleanup."""

from __future__ import annotations

import threading

import pytest

from soarm101_motion.arm import SOARM101
from soarm101_motion.exceptions import MotionCancelledError
from soarm101_motion.motion.controller import PlannedPath


@pytest.mark.parametrize("owned,expected_stop_calls", [(True, 0), (False, 1)])
def test_cancelled_motion_worker_only_issues_unowned_fallback_hold(
    monkeypatch, owned, expected_stop_calls,
) -> None:
    arm = SOARM101.simulated()
    events = []
    monkeypatch.setattr(arm.backend, "stop", lambda: events.append("hold"))
    controller = arm.motion
    cancel = threading.Event()
    cancel.set()
    if owned:
        controller._stop_owned_events.add(cancel)
    plan = PlannedPath(
        joint_waypoints=(), cartesian_waypoints=(),
        command_samples=({"shoulder_pan": 0.0},), duration_s=0.0,
    )
    with pytest.raises(MotionCancelledError):
        controller._execute_plan(plan, cancel, cancellation_message="cancelled")
    assert len(events) == expected_stop_calls


def test_faulted_arm_disconnect_does_not_issue_another_motor_goal(
    monkeypatch,
) -> None:
    arm = SOARM101.simulated()
    events = []

    def unexpected_stop(*args, **kwargs):
        raise AssertionError("never attempt another HOLD on faulted cleanup")

    monkeypatch.setattr(arm.motion, "stop", unexpected_stop)
    monkeypatch.setattr(arm, "_stop_tool", unexpected_stop)
    monkeypatch.setattr(arm.backend, "disconnect", lambda: events.append("port_close"))
    arm.disconnect(stop_before_disconnect=False)
    assert events == ["port_close"]


def test_faulted_broker_close_suppresses_generic_arm_hold_cleanup(
    monkeypatch, tmp_path,
) -> None:
    from soarm101_motion.agent_control import AgentAuthorityStore
    from soarm101_motion.broker import AgentMotionRates
    from soarm101_motion.broker_sdk import SDKAgentExecutor
    from soarm101_motion.config import SOARM101Config

    arm = SOARM101.simulated()
    calls = []
    monkeypatch.setattr(
        arm, "disconnect",
        lambda *, stop_before_disconnect=True:
            calls.append(stop_before_disconnect),
    )
    executor = SDKAgentExecutor(
        config=SOARM101Config(port="/dev/fake", robot_id="so101"),
        rates=AgentMotionRates(), physical_trial=True,
        authority_store=AgentAuthorityStore(tmp_path / "lease.json"),
        arm_factory=lambda: arm,
    )
    executor._session = arm
    executor._lifecycle = "faulted"
    executor.close()
    assert calls == [False]
    assert executor.session_evidence["lifecycle"] == "closed"


def test_verified_physical_stop_close_does_not_reissue_hold(monkeypatch, tmp_path):
    from soarm101_motion.agent_control import AgentAuthorityStore
    from soarm101_motion.broker import AgentMotionRates
    from soarm101_motion.broker_sdk import SDKAgentExecutor
    from soarm101_motion.config import SOARM101Config

    arm = SOARM101.simulated()
    disconnect_options = []
    monkeypatch.setattr(
        arm, "disconnect", lambda *, stop_before_disconnect=True:
        disconnect_options.append(stop_before_disconnect),
    )
    executor = SDKAgentExecutor(
        config=SOARM101Config(port="/dev/fake", robot_id="so101"),
        rates=AgentMotionRates(), physical_trial=True,
        authority_store=AgentAuthorityStore(tmp_path / "lease.json"),
        arm_factory=lambda: arm,
    )
    executor._session = arm
    executor._lifecycle = "open"
    executor._verified_physical_stop = True
    executor.close()
    assert disconnect_options == [False]
    assert executor.session_evidence["lifecycle"] == "closed"


def test_normal_disconnect_uses_controller_cleanup_not_public_stop(monkeypatch) -> None:
    arm = SOARM101.simulated()
    observations = []

    def public_stop_is_broken():
        raise RuntimeError("injected broker STOP failure")

    monkeypatch.setattr(arm, "stop", public_stop_is_broken)
    monkeypatch.setattr(
        arm.motion, "stop", lambda *, wait=True: observations.append("controller"),
    )
    monkeypatch.setattr(
        arm.backend, "disconnect", lambda: observations.append("close"),
    )
    arm.disconnect()
    assert observations == ["controller", "close"]
