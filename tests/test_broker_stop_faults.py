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
from soarm101_motion.exceptions import HardwareFaultError, RobotConnectionError


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
            faulted=self.faulted, fault_message="injected servo fault" if self.faulted else None,
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
    with pytest.raises(RuntimeError, match="gripper STOP failure"):
        arm.stop()
    assert actions == ["arm", "tool"]


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
    saved = [json.loads(l) for l in events.read_text().splitlines()]
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
