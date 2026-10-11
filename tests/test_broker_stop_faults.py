"""Fail-closed STOP and cancellation regressions; never opens physical hardware."""

from __future__ import annotations

import json
import threading
from types import SimpleNamespace

import pytest

from soarm101_motion.agent_control import AgentAuthorityStore
from soarm101_motion.arm import SOARM101
from soarm101_motion.broker import AgentMotionRates, RobotBrokerService
from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.capability_profile import CapabilityProfile
from soarm101_motion.config import SOARM101Config
from soarm101_motion.exceptions import (
    CommunicationError, HardwareFaultError, MotionCancelledError, RobotConnectionError,
)


class FakeStopArm:
    def __init__(self, config):
        self.config = config
        self.calibration_id = "cal-good"
        self.is_connected = False
        self.connect_count = 0
        self.disconnect_count = 0
        self.stop_count = 0
        self.torque_enabled = True
        self.faulted = False

    def connect(self):
        self.is_connected = True
        self.connect_count += 1

    def disconnect(self):
        self.is_connected = False
        self.disconnect_count += 1

    def stop(self):
        self.stop_count += 1

    def get_state(self):
        return SimpleNamespace(
            connected=self.is_connected, torque_enabled=self.torque_enabled,
            faulted=self.faulted, moving=False,
            fault_message="injected servo fault" if self.faulted else None,
        )

    def get_joint_positions(self):
        return SimpleNamespace(positions={"shoulder_pan": 0.0})


def _executor(tmp_path, *, physical=False):
    config = SOARM101Config(
        robot_id="so101", **({"port": "/dev/fake"} if physical else {})
    )
    arm = FakeStopArm(config)
    authority = AgentAuthorityStore(tmp_path / "authority.json")
    executor = SDKAgentExecutor(
        config=config, rates=AgentMotionRates(),
        physical_trial=physical, simulation=not physical,
        arm_factory=lambda: arm, authority_store=authority,
    )
    return executor, arm, authority


def test_stop_attempts_tool_even_when_arm_hold_fails(monkeypatch) -> None:
    arm = SOARM101.simulated()
    actions = []

    def failed_joint_stop(*, wait=True):
        actions.append(("arm", wait))
        raise RuntimeError("injected joint HOLD failure")

    def tool_stop(*, wait=True):
        actions.append(("tool", wait))

    monkeypatch.setattr(arm.motion, "stop", failed_joint_stop)
    monkeypatch.setattr(arm, "_stop_tool", tool_stop)
    with pytest.raises(RuntimeError, match="joint HOLD failure"):
        arm.stop()
    assert actions == [("arm", True), ("tool", True)]


def test_stop_attempts_arm_and_reports_tool_failure(monkeypatch) -> None:
    arm = SOARM101.simulated()
    actions = []

    def arm_stop(*, wait=True):
        actions.append("arm")

    def failed_tool_stop(*, wait=True):
        actions.append("tool")
        raise RuntimeError("injected gripper STOP failure")

    monkeypatch.setattr(arm.motion, "stop", arm_stop)
    monkeypatch.setattr(arm, "_stop_tool", failed_tool_stop)
    # A genuinely active tool still needs independent cancellation.
    arm.tool._active_handle = SimpleNamespace(done=False)
    with pytest.raises(RuntimeError, match="gripper STOP failure"):
        arm.stop()
    assert actions == ["arm", "tool"]


def test_joint_only_stop_does_not_repeat_backend_hold_for_idle_gripper(
    monkeypatch,
) -> None:
    arm = SOARM101.simulated()
    calls = []

    def joint_stop(*, wait=True):
        calls.append("motion")

    def tool_stop(*, wait=True):
        calls.append("tool")

    monkeypatch.setattr(arm.motion, "stop", joint_stop)
    monkeypatch.setattr(arm, "_stop_tool", tool_stop)
    assert not arm.tool.is_moving
    arm.stop()
    assert calls == ["motion"]


def test_native_broker_rejects_inflight_false_success_after_stop(tmp_path, monkeypatch) -> None:
    executor, arm, authority = _executor(tmp_path)
    executor._arm()
    authority.issue(robot_id="so101", calibration_id="simulation", minutes=1)
    started = threading.Event()
    release = threading.Event()
    original = executor._execute_action

    def action(name, request):
        if name == "joint":
            started.set()
            assert release.wait(timeout=3), "simulated joint action never released"
            # Simulate a buggy/laggy transport claiming completion even after STOP.
            return {"accepted": True, "completed": True, "action": "joint"}
        return original(name, request)

    monkeypatch.setattr(executor, "_execute_action", action)
    profile = CapabilityProfile.from_document({
        "schema_version": 1, "name": "stop-false-success",
        "tools": ["jog_joint", "stop", "robot_state"],
        "cameras": [], "limits": {"max_joint_delta_deg": 2},
    })
    events = tmp_path / "events.jsonl"
    service = RobotBrokerService(
        executor=executor, token="local-test-token", profile=profile,
        allow_simulated_sdk_posts=True, event_path=events,
    )
    results = []

    def worker():
        results.append(service.dispatch(
            "POST", "/v1/joint", {"joint": "shoulder_pan", "delta_deg": 0.2},
        ))

    moving = threading.Thread(target=worker)
    moving.start()
    try:
        assert started.wait(timeout=2)
        stopped = service.dispatch("POST", "/v1/stop", {})
        assert stopped.status == 200 and stopped.body["result"]["completed"] is True
        assert arm.stop_count == 1
    finally:
        release.set()
        moving.join(timeout=3)
    assert not moving.is_alive()
    assert len(results) == 1
    assert results[0].status == 409
    assert results[0].body["ok"] is False
    assert "completion is not confirmed" in results[0].body["error"]
    saved = [json.loads(line) for line in events.read_text().splitlines()]
    assert any(e["action"] == "stop" and e["ok"] is True for e in saved)
    assert any(e["action"] == "joint" and e["ok"] is False for e in saved)
    assert not any(e["action"] == "joint" and e["ok"] is True for e in saved)
    assert not authority.status()["armed"]
    executor.close()


def test_stop_bus_failure_faults_session_revokes_lease_and_never_reconnects(tmp_path) -> None:
    executor, arm, authority = _executor(tmp_path, physical=True)
    executor._arm()
    authority.issue(robot_id="so101", calibration_id="cal-good", minutes=1)

    def failed_stop():
        arm.stop_count += 1
        raise RuntimeError("injected STOP/HOLD bus failure")

    arm.stop = failed_stop
    with pytest.raises(RuntimeError, match="STOP/HOLD bus failure"):
        executor.execute("stop", {})
    assert executor.session_evidence["lifecycle"] == "faulted"
    assert executor.session_evidence["fault_kind"] == "RuntimeError"
    assert not authority.status()["armed"]
    with pytest.raises(RuntimeError, match="human restart required"):
        executor.execute("state", {})
    assert arm.connect_count == 1
    executor.close()


def test_stop_cannot_report_completed_from_faulted_readback(tmp_path) -> None:
    executor, arm, _ = _executor(tmp_path)
    executor._arm()
    arm.faulted = True
    with pytest.raises(HardwareFaultError, match="STOP/HOLD unverified"):
        executor.execute("stop", {})
    assert executor.session_evidence["lifecycle"] == "faulted"
    assert arm.stop_count == 1
    executor.close()


def test_stop_rejects_pre_disconnected_device_and_latches_fault(tmp_path) -> None:
    executor, arm, _ = _executor(tmp_path)
    executor._arm()
    arm.is_connected = False
    with pytest.raises(RobotConnectionError, match="device is disconnected"):
        executor.execute("stop", {})
    assert arm.stop_count == 0
    assert executor.session_evidence["lifecycle"] == "faulted"
    executor.close()


def test_native_http_failed_hold_never_acks_or_allows_later_work(tmp_path) -> None:
    executor, arm, authority = _executor(tmp_path)
    executor._arm()
    authority.issue(robot_id="so101", calibration_id="simulation", minutes=1)

    def fail():
        raise RuntimeError("HOLD never confirmed")

    arm.stop = fail
    profile = CapabilityProfile.from_document({
        "schema_version": 1, "name": "stop-fail-closed",
        "tools": ["robot_state", "stop", "jog_joint"],
        "cameras": [], "limits": {"max_joint_delta_deg": 2},
    })
    event_path = tmp_path / "audit.jsonl"
    service = RobotBrokerService(
        executor=executor, token="test", profile=profile,
        allow_simulated_sdk_posts=True, event_path=event_path,
    )
    stopped = service.dispatch("POST", "/v1/stop", {})
    assert stopped.status == 409 and stopped.body["ok"] is False
    assert "HOLD never confirmed" in stopped.body["error"]
    assert not authority.status()["armed"]
    denied = service.dispatch(
        "POST", "/v1/joint", {"joint": "shoulder_pan", "delta_deg": 0.2},
    )
    assert denied.status == 409 and denied.body["ok"] is False
    assert executor.session_evidence["lifecycle"] == "faulted"
    assert arm.connect_count == 1
    entries = [json.loads(x) for x in event_path.read_text().splitlines()]
    assert entries[0]["action"] == "stop" and entries[0]["ok"] is False
    assert entries[0]["result"] == {}
    assert entries[1]["action"] == "joint" and entries[1]["ok"] is False
    executor.close()


@pytest.mark.parametrize("error_type", [
    MotionCancelledError, HardwareFaultError, CommunicationError,
])
def test_sdk_specific_faults_are_authenticated_http_failures_with_audit(
    tmp_path, monkeypatch, error_type,
) -> None:
    """SDK exceptions are not RuntimeError; they must not escape the HTTP worker."""
    executor, arm, _ = _executor(tmp_path)
    executor._arm()

    def fail(action, request):
        raise error_type("injected native SDK error")

    monkeypatch.setattr(executor, "_execute_action", fail)
    profile = CapabilityProfile.from_document({
        "schema_version": 1, "name": "sdk-exception-audit",
        "tools": ["jog_joint", "stop"], "cameras": [],
        "limits": {"max_joint_delta_deg": 2},
    })
    audit = tmp_path / "audit.jsonl"
    service = RobotBrokerService(
        executor=executor, token="test", profile=profile,
        allow_simulated_sdk_posts=True, event_path=audit,
    )
    response = service.dispatch(
        "POST", "/v1/joint", {"joint": "shoulder_pan", "delta_deg": 0.2},
    )
    assert response.status == 409 and response.body["ok"] is False
    assert response.body["request_id"]
    assert "injected native SDK error" in response.body["error"]
    recorded = [json.loads(line) for line in audit.read_text().splitlines()]
    assert len(recorded) == 1
    assert recorded[0]["ok"] is False and recorded[0]["action"] == "joint"
    assert recorded[0]["request_id"] == response.body["request_id"]
    assert recorded[0]["result"] == {}
    if error_type is MotionCancelledError:
        assert executor.session_evidence["lifecycle"] == "open"
    else:
        assert executor.session_evidence["lifecycle"] == "faulted"
    assert arm.connect_count == 1
    executor.close()


def test_physical_stop_rejects_trace_like_encoder_drift_after_ack(
    tmp_path, monkeypatch,
) -> None:
    """Real 2026-10-11 trace moved 18.4 deg AFTER broker returned success."""
    import math
    from types import SimpleNamespace

    import soarm101_motion.broker_sdk as sdk_module

    executor, arm, authority = _executor(tmp_path, physical=True)
    executor._arm()
    authority.issue(robot_id="so101", calibration_id="cal-good", minutes=1)
    sleep_calls = []
    monkeypatch.setattr(
        sdk_module.time, "sleep", lambda delta: sleep_calls.append(delta),
    )
    # 82.8° at STOP; ~64.4° one second afterward. A subset of the
    # extracted trace is reproduced as fake encoder readback, never as
    # a command to the robot.
    samples_deg = iter([82.8, 82.6, 79.2])
    def measured():
        return SimpleNamespace(
            positions={"elbow_flex": math.radians(next(samples_deg))}
        )
    monkeypatch.setattr(arm, "get_joint_positions", measured)
    with pytest.raises(HardwareFaultError, match="moved 3.60"):
        executor.execute("stop", {})
    assert sleep_calls == [0.25, 0.25]
    assert arm.stop_count == 1
    assert executor.session_evidence["lifecycle"] == "faulted"
    assert executor.session_evidence["fault_kind"] == "HardwareFaultError"
    assert not authority.status()["armed"]
    assert arm.connect_count == 1
    with pytest.raises(RuntimeError, match="human restart required"):
        executor.execute("state", {})
    executor.close()


def test_physical_stop_only_acks_stable_post_latch_feedback(
    tmp_path, monkeypatch,
) -> None:
    import math
    from types import SimpleNamespace

    import soarm101_motion.broker_sdk as sdk_module

    executor, arm, authority = _executor(tmp_path, physical=True)
    executor._arm()
    authority.issue(robot_id="so101", calibration_id="cal-good", minutes=1)
    timeline = []
    monkeypatch.setattr(
        sdk_module.time, "sleep", lambda delay: timeline.append(delay),
    )
    angles = iter([50.0, 50.1, 50.05, 50.15, 50.1])
    monkeypatch.setattr(
        arm, "get_joint_positions",
        lambda: SimpleNamespace(
            positions={"elbow_flex": math.radians(next(angles))}
        ),
    )
    result = executor.execute("stop", {})
    assert result["completed"] is True
    assert result["post_stop_stability"]["verified"] is True
    assert len(result["post_stop_stability"]["samples"]) == 4
    assert result["post_stop_stability"]["duration_s"] == pytest.approx(1.5)
    assert result["post_stop_stability"]["max_drift_deg"] == pytest.approx(0.15)
    assert timeline == [0.25, 0.25, 0.5, 0.5]
    assert not authority.status()["armed"]
    executor.close()


def test_physical_stop_rejects_moving_flag_even_with_unchanged_encoder(
    tmp_path, monkeypatch,
) -> None:
    import soarm101_motion.broker_sdk as sdk_module

    executor, arm, _ = _executor(tmp_path, physical=True)
    executor._arm()
    monkeypatch.setattr(sdk_module.time, "sleep", lambda _: None)
    original = arm.get_state
    def state():
        base = original()
        base.moving = True
        return base
    monkeypatch.setattr(arm, "get_state", state)
    with pytest.raises(HardwareFaultError, match="motor state still moving"):
        executor.execute("stop", {})
    assert executor.session_evidence["lifecycle"] == "faulted"
    executor.close()
