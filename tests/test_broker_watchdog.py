"""Fault injection only: these tests never open hardware or arm a real robot."""
from __future__ import annotations

import json
import threading

import pytest

from soarm101_motion.agent_control import AgentAuthorityStore
from soarm101_motion.broker import (
    AgentMotionRates, BrokerRelayService, RobotBrokerHTTPServer, RobotBrokerService,
    validate_physical_agent_policy,
)
from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.broker_watchdog import BrokerWatchdog
from soarm101_motion.capability_profile import CapabilityProfile
from soarm101_motion.config import SOARM101Config


def profile(**changes):
    document = {
        "schema_version": 1, "name": "attended-joint-test",
        "tools": ["robot_health", "robot_capabilities", "robot_state", "jog_joint", "stop"],
        "cameras": [], "limits": {"max_joint_delta_deg": 1},
    }
    document.update(changes)
    return CapabilityProfile.from_document(document)


def test_deadline_cannot_be_revived_by_late_heartbeat():
    now = [0.0]
    trips = []
    watchdog = BrokerWatchdog(timeout_s=1, validate=lambda: None,
                              trip=trips.append, clock=lambda: now[0])
    with pytest.raises(PermissionError, match="heartbeat required"):
        watchdog.check()
    watchdog.heartbeat()
    now[0] = 0.9
    watchdog.heartbeat()
    now[0] = 1.9
    with pytest.raises(PermissionError, match="expired"):
        watchdog.heartbeat()
    assert trips == ["supervisor heartbeat expired"]
    for _ in range(2):
        with pytest.raises(PermissionError, match="terminal"):
            watchdog.heartbeat()
    assert len(trips) == 1


@pytest.mark.parametrize("timeout", [0, -1, 0.49, 11, float("nan"), float("inf")])
def test_invalid_watchdog_deadline(timeout):
    with pytest.raises(ValueError):
        BrokerWatchdog(timeout_s=timeout, validate=lambda: None, trip=lambda reason: None)


@pytest.fixture
def supervised(tmp_path):
    authority = AgentAuthorityStore(tmp_path / "authority.json")
    authority.issue(robot_id="so101", calibration_id="simulation", minutes=1)
    executor = SDKAgentExecutor(
        config=SOARM101Config(), rates=AgentMotionRates(), simulation=True,
        simulation_actions=True, authority_store=authority,
    )
    service = RobotBrokerService(
        executor=executor, token="private", profile=profile(),
        allow_simulated_sdk_posts=True, watchdog_timeout_s=1,
        event_path=tmp_path / "events.jsonl",
    )
    now = [0.0]
    service.watchdog._clock = lambda: now[0]
    assert service.dispatch("GET", "/v1/state").status == 200
    yield service, executor, authority, now
    service.watchdog.close()
    executor.close()


def heartbeat(service):
    return service.dispatch("POST", "/v1/supervisor/heartbeat", {})


def test_motion_requires_host_heartbeat_and_persistent_session(supervised):
    service, executor, authority, now = supervised
    request = {"joint": "shoulder_pan", "delta_deg": 0.1}
    assert service.dispatch("POST", "/v1/joint", request).status == 409
    assert heartbeat(service).status == 200
    for delta in (0.1, -0.1):
        assert service.dispatch("POST", "/v1/joint", {**request, "delta_deg": delta}).status == 200
    assert executor.connection_count == 1
    now[0] = 1.1
    assert heartbeat(service).status == 409
    assert not authority.status()["armed"]
    assert executor.session_evidence["lifecycle"] == "faulted"
    # A new human lease cannot resurrect this terminal broker session.
    authority.issue(robot_id="so101", calibration_id="simulation", minutes=1)
    assert heartbeat(service).status == 409
    assert service.dispatch("POST", "/v1/joint", request).status == 409
    events = [json.loads(line) for line in service.event_path.read_text().splitlines()]
    assert len([event for event in events if event["action"] == "watchdog_trip"]) == 1


@pytest.mark.parametrize("change", ["clear", "expire", "replace", "corrupt"])
def test_simulation_watchdog_does_not_depend_on_human_motion_lease(supervised, change):
    service, executor, authority, now = supervised
    assert heartbeat(service).status == 200
    if change == "clear":
        authority.clear()
    elif change == "expire":
        lease = authority.load()
        authority.path.write_text(json.dumps({**lease.__dict__, "expires_at": lease.issued_at + 0.001,
                                              "issued_at": lease.issued_at - 120}))
        # Expire relative to wall clock, independent of monotonic supervision.
        import time
        document = json.loads(authority.path.read_text())
        document["expires_at"] = time.time() - 1
        authority.path.write_text(json.dumps(document))
    elif change == "replace":
        authority.issue(robot_id="so101", calibration_id="simulation", minutes=2)
    elif change == "corrupt":
        authority.path.write_text("not json")
    assert heartbeat(service).status == 200
    assert service.watchdog.status["state"] == "watching"


def test_simulation_watchdog_still_trips_on_session_fault(supervised):
    service, executor, authority, _now = supervised
    assert heartbeat(service).status == 200
    executor._fault(OSError("injected transport failure"))
    assert heartbeat(service).status == 409
    assert service.watchdog.status["state"] == "tripped"
    assert not authority.path.exists()


def test_private_heartbeat_never_reaches_agent_relay(supervised):
    service, _, _, _ = supervised
    relay = BrokerRelayService(backend=service, token="agent", agent_profile=profile())
    for method, path in (("POST", "/v1/supervisor/heartbeat"),
                         ("GET", "/v1/supervisor/watchdog")):
        assert relay.dispatch(method, path, {}).status == 403
    assert relay.authorized("Bearer private") is False
    assert service.authorized("Bearer agent") is False
    assert service.watchdog.status["state"] == "waiting"


def test_explicit_stop_finishes_watchdog_without_second_latch(supervised, monkeypatch):
    service, executor, authority, now = supervised
    assert heartbeat(service).status == 200
    executor._session.enable()  # Simulated torque only.
    calls = []
    original = executor._session.stop
    def stop():
        calls.append("stop")
        return original()
    monkeypatch.setattr(executor._session, "stop", stop)
    assert service.dispatch("POST", "/v1/stop", {}).status == 200
    now[0] = 10
    with pytest.raises(PermissionError):
        service.watchdog.check()
    assert heartbeat(service).status == 409
    assert service.watchdog.status["state"] == "stopped"
    assert service.watchdog.status["stop_result"]["holding"] is True
    assert calls == ["stop"]


def test_watchdog_interrupts_inflight_action_and_rejects_queued_and_late_success(
    supervised, monkeypatch,
):
    service, executor, _, now = supervised
    assert heartbeat(service).status == 200
    entered, released = threading.Event(), threading.Event()
    original = executor._execute_action
    calls = []
    def execute(action, request):
        if action == "joint":
            calls.append("joint")
            entered.set()
            assert released.wait(3), "STOP waited behind the action lock"
            return {"accepted": True, "completed": True, "holding": True}
        if action == "stop":
            calls.append("stop")
            released.set()
        return original(action, request)
    monkeypatch.setattr(executor, "_execute_action", execute)
    responses = []
    def move():
        responses.append(service.dispatch("POST", "/v1/joint", {
            "joint": "shoulder_pan", "delta_deg": 0.1,
        }))
    active = threading.Thread(target=move)
    queued = threading.Thread(target=move)
    active.start()
    assert entered.wait(1)
    queued.start()
    now[0] = 2
    try:
        with pytest.raises(PermissionError):
            service.watchdog.check()
    finally:
        released.set()
        active.join(3)
        queued.join(3)
    assert not active.is_alive() and not queued.is_alive()
    assert calls == ["joint", "stop"]
    assert [response.status for response in responses] == [409, 409]


def test_failed_stop_is_terminal_audited_and_not_retried(supervised, monkeypatch):
    service, executor, authority, now = supervised
    assert heartbeat(service).status == 200
    calls = []
    def fail():
        calls.append("stop")
        raise RuntimeError("injected HOLD failure")
    monkeypatch.setattr(executor._session, "stop", fail)
    now[0] = 2
    with pytest.raises(PermissionError):
        service.watchdog.check()
    for _ in range(2):
        assert heartbeat(service).status == 409
    assert calls == ["stop"]
    assert service.watchdog.status["stop_error"] == "injected HOLD failure"
    assert not authority.status()["armed"]
    events = [json.loads(line) for line in service.event_path.read_text().splitlines()]
    assert events[-1]["action"] == "watchdog_trip" and events[-1]["ok"] is False


@pytest.mark.parametrize("changes", [
    {"tools": ["robot_state", "robot_capabilities", "jog_joint", "jog_cartesian"]},
    {"limits": {}}, {"limits": {"max_joint_delta_deg": 3}},
])
def test_physical_profile_cannot_expand_experimental_motion(changes):
    with pytest.raises(ValueError):
        validate_physical_agent_policy(profile(**changes), AgentMotionRates())


def test_physical_broker_requires_watchdog_and_loopback(tmp_path):
    config = SOARM101Config(port="/dev/FAKE", disable_torque_on_disconnect=False)
    executor = SDKAgentExecutor(config=config, rates=AgentMotionRates(), physical_actions=True,
                                arm_factory=lambda: pytest.fail("must not connect"))
    with pytest.raises(ValueError, match="watchdog"):
        RobotBrokerService(executor=executor, token="host", profile=profile(),
                           allow_physical_sdk_actions=True)
    service = RobotBrokerService(executor=executor, token="host", profile=profile(),
                                 allow_physical_sdk_actions=True, watchdog_timeout_s=1)
    with pytest.raises(ValueError, match="loopback"):
        RobotBrokerHTTPServer(("0.0.0.0", 0), service)
    for action in ("jog", "go_pose", "sleep", "gripper"):
        with pytest.raises(PermissionError):
            executor.execute(action, {})
    service.watchdog.close()
    executor.close()


def test_background_watchdog_stops_without_another_request():
    stopped = threading.Event()
    watchdog = BrokerWatchdog(timeout_s=0.5, validate=lambda: None,
                              trip=lambda reason: stopped.set())
    watchdog.heartbeat()
    watchdog.start()
    try:
        assert stopped.wait(2)
        assert watchdog.status["state"] == "tripped"
    finally:
        watchdog.close()


def test_revocation_io_failure_does_not_prevent_stop_attempt(supervised, monkeypatch):
    service, executor, authority, now = supervised
    assert heartbeat(service).status == 200
    stopped = []
    original = executor._session.stop
    def stop():
        stopped.append(True)
        return original()
    def refuse_clear():
        raise OSError("lease filesystem unavailable")
    monkeypatch.setattr(executor._session, "stop", stop)
    monkeypatch.setattr(authority, "clear", refuse_clear)
    now[0] = 2
    with pytest.raises(PermissionError):
        service.watchdog.check()
    assert stopped == [True]
    assert executor.session_evidence["lifecycle"] == "faulted"
    assert "revocation failed" in service.watchdog.status["stop_error"]


def test_second_stop_preserves_first_failed_outcome(supervised, monkeypatch):
    service, executor, _, _ = supervised
    assert heartbeat(service).status == 200
    calls = []
    def fail():
        calls.append(True)
        raise RuntimeError("first HOLD failed")
    monkeypatch.setattr(executor._session, "stop", fail)
    assert service.dispatch("POST", "/v1/stop", {}).status == 409
    assert service.dispatch("POST", "/v1/stop", {}).status == 409
    assert calls == [True]
    assert service.watchdog.status["stop_error"] == "first HOLD failed"


def test_physical_joint_path_reuses_human_hold_and_checks_complete_result(tmp_path, monkeypatch):
    from types import SimpleNamespace
    import soarm101_motion.broker_sdk as broker_sdk
    monkeypatch.setattr(broker_sdk, "STOP_STABILITY_SAMPLE_DELAYS_S", (0,))
    config = SOARM101Config(port="/dev/FAKE", disable_torque_on_disconnect=False)
    authority = AgentAuthorityStore(tmp_path / "lease.json")
    authority.issue(robot_id="so101", calibration_id="cal-test", minutes=1)
    class Arm:
        calibration_id = "cal-test"
        is_connected = False
        torque = True
        position = 0.0
        stop_calls = 0
        def connect(self):
            self.is_connected = True
        def disconnect(self):
            self.is_connected = False
        def enable(self):
            pytest.fail("physical actions must reuse human HOLD, never enable torque")
        def hold(self):
            pytest.fail("do not re-latch or secretly enable torque after a joint action")
        def get_state(self):
            return SimpleNamespace(connected=True, torque_enabled=self.torque,
                                   faulted=False, moving=False)
        def get_joint_positions(self):
            return SimpleNamespace(positions={"shoulder_pan": self.position})
        def move_joints(self, positions, **kwargs):
            assert kwargs["relative"] is True
            self.position += positions["shoulder_pan"]
            return SimpleNamespace(accepted=True, completed=True, message="fake complete")
        def stop(self):
            self.stop_calls += 1
    arm = Arm()
    arm.config = config
    executor = SDKAgentExecutor(config=config, rates=AgentMotionRates(), physical_actions=True,
                                authority_store=authority, arm_factory=lambda: arm)
    service = RobotBrokerService(
        executor=executor, token="host", profile=profile(), event_path=tmp_path / "events.jsonl",
        allow_physical_sdk_actions=True, watchdog_timeout_s=1,
    )
    try:
        executor._arm()  # Fake read-only connect; no physical device.
        assert heartbeat(service).status == 200
        for delta in (0.1, -0.1):
            assert service.dispatch("POST", "/v1/joint", {
                "joint": "shoulder_pan", "delta_deg": delta,
            }).status == 200
        assert executor.connection_count == 1
        assert service.dispatch("POST", "/v1/joint", {
            "joint": "shoulder_pan", "delta_deg": 2,
        }).status == 403  # Profile remains stricter than the rollout ceiling.
        arm.torque = False
        assert service.dispatch("POST", "/v1/joint", {
            "joint": "shoulder_pan", "delta_deg": 0.1,
        }).status == 409
        assert executor.session_evidence["lifecycle"] == "faulted"
    finally:
        service.watchdog.close()
        executor.close()


def test_real_broker_subprocess_stops_when_supervisor_heartbeats_cease(tmp_path, monkeypatch):
    import socket
    import time
    import urllib.request
    from soarm101_motion.agent_sandbox import AgentSandboxError, BrokerProcess
    authority = AgentAuthorityStore(tmp_path / "simulation-authority.json")
    authority.issue(robot_id="so101", calibration_id="simulation", minutes=1)
    monkeypatch.setenv("SOARM101_AGENT_AUTHORITY_PATH", str(authority.path))
    file = tmp_path / "profile.json"
    file.write_text(json.dumps(profile().as_dict()))
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    broker = BrokerProcess(port=port, token="agent-only", event_path=tmp_path / "events.jsonl",
                           profile_path=file, sdk_simulation=True)
    try:
        broker.start()
        broker.require_authority()
        broker.request("GET", "/v1/state")
        broker.start_supervision()
        request = urllib.request.Request(
            f"http://127.0.0.1:{port}/v1/joint",
            data=json.dumps({"joint": "shoulder_pan", "delta_deg": 0.1}).encode(),
            headers={"Authorization": "Bearer agent-only", "Content-Type": "application/json"},
        )
        with urllib.request.urlopen(request, timeout=3) as response:
            assert json.load(response)["ok"]
        # Lose the host heartbeat task while leaving the broker alive, exactly
        # the condition finally/normal teardown alone cannot handle.
        broker._heartbeat_closed.set()
        broker._heartbeat_worker.join(2)
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            status = broker.request("GET", "/v1/supervisor/watchdog")["result"]
            if status["state"] == "tripped" and status["stop_result"] is not None:
                break
            time.sleep(0.05)
        assert status["state"] == "tripped"
        assert status["stop_result"]["holding"] is True
        assert not authority.path.exists()
        with pytest.raises(AgentSandboxError, match="watchdog tripped"):
            broker.request_motion_stop()
        events = [json.loads(line) for line in broker.event_path.read_text().splitlines()]
        assert len([event for event in events if event["action"] == "stop"]) == 1
    finally:
        broker.stop()


@pytest.mark.parametrize("phase", ["facade_preflight", "controller_planning"])
def test_supervised_stop_fences_motion_still_planning(supervised, monkeypatch, phase):
    service, executor, _, _ = supervised
    assert heartbeat(service).status == 200
    arm = executor._session
    arm.enable()  # Simulation only.
    entered, release = threading.Event(), threading.Event()
    name = "_validate_joint_workspace_path" if phase == "facade_preflight" else "_plan_joint_motion"
    owner = arm if phase == "facade_preflight" else arm.motion
    original = getattr(owner, name)
    writes = []
    original_write = arm.backend.write_joint_positions
    def write(*args, **kwargs):
        writes.append(True)
        return original_write(*args, **kwargs)
    def plan(*args, **kwargs):
        entered.set()
        assert release.wait(3)
        return original(*args, **kwargs)
    monkeypatch.setattr(owner, name, plan)
    monkeypatch.setattr(arm.backend, "write_joint_positions", write)
    results = []
    movement = threading.Thread(target=lambda: results.append(service.dispatch("POST", "/v1/joint", {
        "joint": "shoulder_pan", "delta_deg": 0.1,
    })))
    movement.start()
    assert entered.wait(2)
    stop_results = []
    stopper = threading.Thread(target=lambda: stop_results.append(service.dispatch("POST", "/v1/stop", {})))
    stopper.start()
    # Planning may own the state lock, so STOP waits to inspect the handle.
    # Its independent terminal fence must be visible before planning returns.
    assert arm.motion._motion_inhibited.wait(2)
    release.set()
    movement.join(3)
    stopper.join(3)
    assert not movement.is_alive() and not stopper.is_alive()
    assert results[0].status == 409
    assert stop_results[0].status == 200
    assert writes == []
    assert arm.motion.is_moving is False
