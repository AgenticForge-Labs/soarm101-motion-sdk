"""Simulation-only tests for the staged direct SDK broker executor."""

from __future__ import annotations

import base64
import hashlib
import json
import threading
import urllib.error
import urllib.request
from types import SimpleNamespace

import pytest

from soarm101_motion.agent_control import AgentAuthorityStore
from soarm101_motion.broker import (
    AgentMotionRates, RobotBrokerHTTPServer, RobotBrokerService, build_parser,
)
from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.capability_profile import CapabilityProfile
from soarm101_motion.config import SOARM101Config


@pytest.fixture
def native(tmp_path):
    auth = AgentAuthorityStore(tmp_path / "human-lease.json")
    executor = SDKAgentExecutor(
        config=SOARM101Config(robot_id="so101"),
        rates=AgentMotionRates(),
        simulation=True,
        authority_store=auth,
    )
    yield executor, auth
    executor.close()


def test_hardware_direct_sdk_execution_is_not_available_by_default() -> None:
    with pytest.raises(ValueError, match="disabled pending supervised"):
        SDKAgentExecutor(
            config=SOARM101Config(robot_id="so101"),
            rates=AgentMotionRates(),
        )


def test_simulation_session_is_lazy_persistent_and_read_only_by_default(native) -> None:
    executor, auth = native
    assert executor._session is None
    capabilities = executor.execute("capabilities", {})
    assert capabilities["actions"] == {"state": "read_only"}
    assert capabilities["authority"]["armed"] is False
    assert capabilities["world_directions"]["available"] is False
    assert executor._session is None
    result = executor.execute("state", {})
    assert result["robot_id"] == "so101"
    assert result["calibration_id"] == "simulation"
    assert len(result["tcp_xyz_mm"]) == 3
    first = executor._session
    assert first is not None
    assert first.get_state().torque_enabled is False
    assert executor.execute("state", {})["robot_id"] == "so101"
    assert executor._session is first
    assert auth.status()["armed"] is False


def test_unknown_parameters_and_unsafe_actions_fail_before_session(native) -> None:
    executor, _ = native
    for action, request in (
        ("joint", {"joint": "shoulder_pan", "delta_deg": 1, "speed_deg_s": 100}),
        ("state", {"port": "/dev/ttyACM0"}),
        ("capture", {"camera": "overhead", "output": "/tmp/unsafe.jpg"}),
        ("raw_servos", {"value": [1, 2, 3]}),
        ("sleep", {"calibration": "overridden"}),
    ):
        with pytest.raises(ValueError):
            executor.execute(action, request)
    assert executor._session is None


def test_no_lease_cannot_enable_torque(native) -> None:
    executor, _ = native
    with pytest.raises(PermissionError, match="not armed"):
        executor.execute("joint", {"joint": "shoulder_pan", "delta_deg": 1.0})
    assert executor._session is not None
    assert executor._session.get_state().torque_enabled is False


def test_stop_does_not_implicitly_open_or_enable_robot(native) -> None:
    executor, _ = native
    with pytest.raises(RuntimeError, match="no connected SDK session"):
        executor.execute("stop", {})
    assert executor._session is None
    executor.execute("state", {})
    response = executor.execute("stop", {})
    assert response["action"] == "stop"
    assert response["completed"] is True
    assert executor._session.get_state().torque_enabled is False


def test_preview_rejects_physical_jog_without_measured_workspace(native) -> None:
    executor, auth = native
    auth.issue(robot_id="so101", calibration_id="simulation", minutes=1)
    with pytest.raises(PermissionError, match="measured workspace"):
        executor.execute(
            "jog", {"frame": "world", "x_mm": 1, "y_mm": 0, "z_mm": 0},
        )
    assert executor._session.get_state().torque_enabled is False


def test_expired_authority_refuses_motion(native) -> None:
    executor, auth = native
    auth.issue(robot_id="so101", calibration_id="simulation", minutes=1,
               now=1.0)
    with pytest.raises(PermissionError, match="expired"):
        executor.execute("joint", {"joint": "shoulder_pan", "delta_deg": 2.0})
    assert executor._session.get_state().torque_enabled is False


def test_invalid_joint_and_bool_delta_are_rejected(native) -> None:
    executor, auth = native
    auth.issue(robot_id="so101", calibration_id="simulation", minutes=1)
    with pytest.raises(ValueError, match="joint"):
        executor.execute("joint", {"joint": "motor_6", "delta_deg": 1.0})
    with pytest.raises(ValueError, match="finite number"):
        executor.execute("joint", {"joint": "shoulder_pan", "delta_deg": True})
    assert executor._session.get_state().torque_enabled is False


def test_opt_in_native_broker_exposes_only_read_routes(tmp_path) -> None:
    executor = SDKAgentExecutor(
        config=SOARM101Config(robot_id="so101"),
        rates=AgentMotionRates(),
        simulation=True,
        authority_store=AgentAuthorityStore(tmp_path / "lease.json"),
    )
    profile = CapabilityProfile.from_document({
        "schema_version": 1,
        "name": "sdk-preview-read-only",
        "tools": ["robot_health", "robot_state", "robot_capabilities"],
        "cameras": [],
        "limits": {},
    })
    service = RobotBrokerService(
        executor=executor,
        token="sandbox-secret",
        profile=profile,
        event_path=tmp_path / "broker-events.jsonl",
    )
    try:
        assert service.authorized("Bearer sandbox-secret")
        assert service.dispatch("GET", "/v1/health").status == 200
        state = service.dispatch("GET", "/v1/state")
        assert state.status == 200
        assert state.body["result"]["calibration_id"] == "simulation"
        caps = service.dispatch("GET", "/v1/capabilities")
        assert caps.status == 200
        visible = caps.body["result"]["actions"]
        assert "state" in visible
        assert "joint" not in visible and "jog" not in visible
        for endpoint, payload in (
            ("/v1/joint", {"joint": "shoulder_pan", "delta_deg": 1}),
            ("/v1/jog", {"frame": "world", "x_mm": 2}),
            ("/v1/stop", {}),
            ("/v1/capture", {"camera": "overhead"}),
            ("/v1/arm", {}),
        ):
            response = service.dispatch("POST", endpoint, payload)
            assert response.status in (403, 404)
            assert response.body["ok"] is False
        assert executor._session.get_state().torque_enabled is False
        expected = service.profile.public()["sha256"]
        events = [
            __import__("json").loads(line)
            for line in (tmp_path / "broker-events.jsonl").read_text().splitlines()
        ]
        assert events and all(row["profile_sha256"] == expected for row in events)
    finally:
        executor.close()


def test_native_preview_parser_is_opt_in() -> None:
    assert build_parser().parse_args([]).sdk_simulation_preview is False
    assert build_parser().parse_args(["--sdk-simulation-preview"]).sdk_simulation_preview



def test_broker_cli_sdk_simulation_actions_keep_one_session_and_stop(
    tmp_path, monkeypatch,
) -> None:
    """Real broker CLI composition, no subprocesses, bus or physical connection."""
    import soarm101_motion.broker as broker_module

    authority_file = tmp_path / "authority.json"
    monkeypatch.setenv("SOARM101_AGENT_AUTHORITY_PATH", str(authority_file))
    monkeypatch.setenv("SOARM101_BROKER_TOKEN", "synthetic-test-only-token")
    lease = AgentAuthorityStore(authority_file)
    lease.issue(robot_id="so101", calibration_id="simulation", minutes=5)

    doc = {
        "schema_version": 1, "name": "simulated-small-joint",
        "tools": ["robot_health", "robot_capabilities", "robot_state",
                  "jog_joint", "stop"],
        "cameras": [], "limits": {"max_joint_delta_deg": 1},
    }
    profile_path = tmp_path / "profile.json"
    profile_path.write_text(json.dumps(doc), encoding="utf-8")
    event_path = tmp_path / "events.jsonl"
    events = []
    class FakeServer:
        def __init__(self, address, service):
            assert address[0] == "127.0.0.1"
            self.service = service

        def serve_forever(self):
            service = self.service
            assert isinstance(service.executor, SDKAgentExecutor)
            assert service.executor.simulation is True
            assert service.executor.physical_trial is False
            assert service.executor.simulation_actions is True
            assert service.executor._session is None
            caps = service.dispatch("GET", "/v1/capabilities")
            assert caps.status == 200
            advertised = caps.body["result"]["actions"]
            assert "joint" in advertised and "stop" in advertised
            assert "go_pose" not in advertised and "jog" not in advertised
            assert caps.body["result"]["authority"]["armed"] is True
            first = service.dispatch("GET", "/v1/state")
            assert first.status == 200
            assert first.body["result"]["broker_session"]["sdk_connect_count"] == 1
            identity = first.body["result"]["broker_session"]["id"]
            for delta in (0.2, -0.2):
                result = service.dispatch(
                    "POST", "/v1/joint",
                    {"joint": "shoulder_pan", "delta_deg": delta},
                )
                assert result.status == 200, result.body
                assert result.body["result"]["completed"] is True
                assert result.body["result"]["broker_session"]["id"] == identity
                assert result.body["result"]["broker_session"]["sdk_connect_count"] == 1
            assert service.dispatch("POST", "/v1/stop", {}).status == 200
            assert not lease.status()["armed"]
            later = service.dispatch(
                "POST", "/v1/joint",
                {"joint": "shoulder_pan", "delta_deg": 0.1},
            )
            assert later.status == 409
            assert "not armed" in later.body["error"]
            events.extend(json.loads(line) for line in event_path.read_text().splitlines())

        def server_close(self):
            pass

    monkeypatch.setattr(broker_module, "RobotBrokerHTTPServer", FakeServer)
    monkeypatch.setattr(
        broker_module.subprocess, "run",
        lambda *a, **k: pytest.fail("native simulation must not launch CLI subprocess"),
    )
    assert broker_module.main([
        "--sdk-simulation-actions", "--profile", str(profile_path),
        "--events", str(event_path), "--port", "0",
    ]) == 0
    assert any(x["action"] == "stop" and x["ok"] for x in events)
    assert len([x for x in events if x["action"] == "joint" and x["ok"]]) == 2
    assert len({x["profile_sha256"] for x in events}) == 1


def test_cli_sdk_simulation_actions_fail_closed_before_server(
    tmp_path, monkeypatch,
) -> None:
    import soarm101_motion.broker as broker_module
    monkeypatch.setenv("SOARM101_BROKER_TOKEN", "test-only-token")
    monkeypatch.setattr(
        broker_module, "RobotBrokerHTTPServer",
        lambda *a, **kw: pytest.fail("unsafe broker must not be started"),
    )
    profile = tmp_path / "profile.json"
    cases = [
        (None, "requires an explicit --profile", []),
        ({"tools": ["robot_state", "jog_joint", "stop"]},
         "requires capabilities, state and STOP", []),
        ({"tools": ["robot_state", "robot_capabilities", "jog_joint"]},
         "requires capabilities, state and STOP", []),
        ({"tools": ["robot_state", "robot_capabilities", "stop", "capture_camera"],
          "cameras": ["overhead"]}, "cannot authorize cameras", []),
        ({"tools": ["robot_state", "robot_capabilities", "stop", "jog_cartesian"]},
         "cannot authorize cameras or Cartesian jogs", []),
        ({"tools": ["robot_state", "robot_capabilities", "stop", "jog_joint"]},
         "must bind only to 127.0.0.1", ["--host", "0.0.0.0"]),
    ]
    for spec, fragment, args in cases:
        if spec is None:
            argv = ["--sdk-simulation-actions"]
        else:
            doc = {
                "schema_version": 1, "name": "negative", "cameras": [],
                "limits": {}, **spec,
            }
            profile.write_text(json.dumps(doc), encoding="utf-8")
            argv = ["--sdk-simulation-actions", "--profile", str(profile), *args]
        with pytest.raises(SystemExit, match=fragment):
            broker_module.main(argv)


def test_simulated_native_broker_cannot_listen_on_external_interface(tmp_path) -> None:
    executor = SDKAgentExecutor(
        config=SOARM101Config(robot_id="so101"),
        rates=AgentMotionRates(), simulation=True, simulation_actions=True,
        authority_store=AgentAuthorityStore(tmp_path / "authority.json"),
    )
    service = RobotBrokerService(
        executor=executor, token="simulated-token",
        profile=CapabilityProfile.from_document({
            "schema_version": 1, "name": "simulation",
            "tools": ["robot_state", "jog_joint", "stop"],
            "cameras": [], "limits": {"max_joint_delta_deg": 1},
        }),
        allow_simulated_sdk_posts=True,
    )
    with pytest.raises(ValueError, match="loopback"):
        RobotBrokerHTTPServer(("0.0.0.0", 0), service)
    executor.close()


def test_simulation_action_mode_cannot_enable_real_arm() -> None:
    with pytest.raises(ValueError, match="simulated arm only"):
        SDKAgentExecutor(
            config=SOARM101Config(robot_id="so101"),
            rates=AgentMotionRates(), simulation_actions=True,
        )


def test_agent_relay_scopes_bearer_and_preserves_host_only_stop(
    tmp_path,
) -> None:
    """Real HTTP relay and private SDK broker; no hardware or agent models."""
    from soarm101_motion.broker import (
        BrokerRelayHTTPServer, BrokerRelayService,
    )
    auth = AgentAuthorityStore(tmp_path / "simulation-lease.json")
    auth.issue(robot_id="so101", calibration_id="simulation", minutes=5)
    agent = CapabilityProfile.from_document({
        "schema_version": 1, "name": "agent-no-stop",
        "tools": ["robot_health", "robot_capabilities", "robot_state",
                  "jog_joint"],
        "cameras": [], "limits": {"max_joint_delta_deg": 0.5},
    })
    backend_doc = agent.as_dict()
    backend_doc["tools"] = sorted(agent.allowed_tools | {"stop"})
    backend = SDKAgentExecutor(
        config=SOARM101Config(robot_id="so101"),
        rates=AgentMotionRates(), simulation=True, simulation_actions=True,
        authority_store=auth,
    )
    controller = RobotBrokerService(
        executor=backend, token="trusted-host-secret",
        profile=CapabilityProfile.from_document(backend_doc),
        allow_simulated_sdk_posts=True, event_path=tmp_path / "events.jsonl",
    )
    host = RobotBrokerHTTPServer(("127.0.0.1", 0), controller)
    relay = BrokerRelayHTTPServer(
        ("127.0.0.1", 0),
        BrokerRelayService(
            backend=controller, token="agent-only-secret", agent_profile=agent,
        ),
    )
    threads = [
        threading.Thread(target=server.serve_forever, daemon=True)
        for server in (host, relay)
    ]
    for thread in threads:
        thread.start()

    def call(server, method, path, token, payload=None):
        data = json.dumps(payload).encode("utf-8") if payload is not None else None
        req = urllib.request.Request(
            f"http://127.0.0.1:{server.server_address[1]}{path}",
            data=data, method=method,
            headers={
                "Authorization": f"Bearer {token}",
                "Content-Type": "application/json",
            },
        )
        try:
            with urllib.request.urlopen(req, timeout=3) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read())
    try:
        assert call(relay, "GET", "/v1/health", "trusted-host-secret")[0] == 401
        assert call(host, "GET", "/v1/health", "agent-only-secret")[0] == 401
        code, profile = call(relay, "GET", "/v1/profile", "agent-only-secret")
        assert code == 200 and "stop" not in profile["result"]["tools"]
        visible = call(relay, "GET", "/v1/capabilities", "agent-only-secret")
        assert visible[0] == 200
        assert "stop" not in visible[1]["result"]["actions"]
        assert "joint" in visible[1]["result"]["actions"]
        assert call(relay, "POST", "/v1/stop", "agent-only-secret", {})[0] == 403
        assert call(host, "GET", "/v1/state", "trusted-host-secret")[0] == 200
        first = backend.session_evidence["id"]
        assert call(
            relay, "POST", "/v1/joint", "agent-only-secret",
            {"joint": "shoulder_pan", "delta_deg": 0.2},
        )[0] == 200
        assert call(
            relay, "POST", "/v1/joint", "agent-only-secret",
            {"joint": "shoulder_pan", "delta_deg": -0.2},
        )[0] == 200
        assert backend.session_evidence["id"] == first
        assert backend.connection_count == 1
        assert call(
            relay, "POST", "/v1/joint", "agent-only-secret",
            {"joint": "shoulder_pan", "delta_deg": 2.0},
        )[0] == 403
        assert call(host, "POST", "/v1/stop", "trusted-host-secret", {})[0] == 200
        assert auth.status()["armed"] is False
        assert call(
            relay, "POST", "/v1/joint", "agent-only-secret",
            {"joint": "shoulder_pan", "delta_deg": 0.2},
        )[0] == 409
    finally:
        for server in (relay, host):
            server.shutdown()
            server.server_close()
        for thread in threads:
            thread.join(timeout=3)
        backend.close()


def test_relay_denies_equal_tokens_and_physical_mode(tmp_path) -> None:
    from soarm101_motion.broker import BrokerRelayService

    executor = SDKAgentExecutor(
        config=SOARM101Config(robot_id="so101"),
        rates=AgentMotionRates(), simulation=True,
    )
    backend = RobotBrokerService(executor=executor, token="secret")
    profile = CapabilityProfile.from_document({
        "schema_version": 1, "name": "ro",
        "tools": ["robot_state"], "cameras": [], "limits": {},
    })
    with pytest.raises(ValueError, match="separate"):
        BrokerRelayService(
            backend=backend, token="secret", agent_profile=profile,
        )
    executor.close()


def test_native_executor_stop_can_interrupt_a_fake_inflight_move(tmp_path) -> None:
    """Executor unit proof only; NOT a real serial/HOLD or HTTP-level STOP proof."""
    started = threading.Event()
    interrupted = threading.Event()
    config = SOARM101Config(robot_id="so101")

    class FakeArm:
        calibration_id = None

        def __init__(self):
            self.config = config

        def connect(self):
            pass

        def disconnect(self):
            pass

        def get_joint_positions(self):
            return SimpleNamespace(positions={"shoulder_pan": 0.0})

        def get_state(self):
            return SimpleNamespace(torque_enabled=True)

        def enable(self):
            pass

        def move_joints(self, positions, *, relative, speed, acceleration):
            assert positions == {"shoulder_pan": pytest.approx(0.02 * 3.141592653589793 / 180)}
            assert relative
            started.set()
            if not interrupted.wait(timeout=2):
                raise RuntimeError("STOP was blocked behind motion")
            return SimpleNamespace(accepted=True, completed=False, message="interrupted")

        def stop(self):
            interrupted.set()

        def hold(self):
            pass

    lease = AgentAuthorityStore(tmp_path / "lease.json")
    lease.issue(robot_id="so101", calibration_id="simulation", minutes=1)
    executor = SDKAgentExecutor(
        config=config,
        rates=AgentMotionRates(),
        simulation=True,
        arm_factory=FakeArm,
        authority_store=lease,
    )
    errors = []

    def moving_worker():
        try:
            executor.execute(
                "joint", {"joint": "shoulder_pan", "delta_deg": 0.02},
            )
        except Exception as exc:
            errors.append(exc)

    thread = threading.Thread(target=moving_worker)
    thread.start()
    try:
        assert started.wait(timeout=2)
        stopped = executor.execute("stop", {})
        assert stopped["action"] == "stop"
        assert stopped["holding"] is True
        assert interrupted.is_set()
        thread.join(timeout=2)
        assert not thread.is_alive()
        assert len(errors) == 1
        assert "did not complete" in str(errors[0])
    finally:
        interrupted.set()
        thread.join(timeout=2)
        executor.close()


def test_native_executor_serializes_ordinary_calls_without_blocking_stop(
    native, monkeypatch,
) -> None:
    executor, _ = native
    entered = threading.Event()
    release = threading.Event()
    observations = []
    errors = []

    def guarded_action(action, request):
        assert action == "state"
        observations.append("enter")
        if len(observations) == 1:
            entered.set()
            assert release.wait(timeout=2)
        return {"action": action}

    monkeypatch.setattr(executor, "_execute_action", guarded_action)

    def call_state():
        try:
            executor.execute("state", {})
        except Exception as exc:
            errors.append(exc)

    first = threading.Thread(target=call_state)
    second = threading.Thread(target=call_state)
    first.start()
    try:
        assert entered.wait(timeout=2)
        second.start()
        assert not release.is_set()
        assert observations == ["enter"]
    finally:
        release.set()
        first.join(timeout=2)
        if second.ident is not None:
            second.join(timeout=2)
    assert not errors
    assert observations == ["enter", "enter"]
    assert not first.is_alive() and not second.is_alive()


def test_preview_sdk_disconnect_error_is_audited_not_http_success(
    native, tmp_path, monkeypatch,
) -> None:
    executor, _ = native
    profile = CapabilityProfile.from_document({
        "schema_version": 1, "name": "read-only",
        "tools": ["robot_state"], "cameras": [], "limits": {},
    })
    service = RobotBrokerService(
        executor=executor,
        token="token",
        profile=profile,
        event_path=tmp_path / "audit.jsonl",
    )

    def disconnected(action, request):
        raise RuntimeError("SDK session disconnected")

    monkeypatch.setattr(executor, "execute", disconnected)
    response = service.dispatch("GET", "/v1/state")
    assert response.status == 409
    assert response.body["ok"] is False
    records = [
        __import__("json").loads(line)
        for line in (tmp_path / "audit.jsonl").read_text().splitlines()
    ]
    assert len(records) == 1
    assert records[0]["ok"] is False
    assert records[0]["profile_sha256"] == profile.public()["sha256"]



def test_simulated_native_posts_reject_real_executor_and_default_preview(native, tmp_path) -> None:
    executor, _ = native
    profile = CapabilityProfile.from_document({
        "schema_version": 1, "name": "limited-simulation",
        "tools": ["robot_state", "jog_joint", "stop"],
        "cameras": [], "limits": {"max_joint_delta_deg": 0.5},
    })
    with pytest.raises(ValueError, match="explicitly simulated"):
        RobotBrokerService(
            executor=None, token="secret", profile=profile,
            allow_simulated_sdk_posts=True,
        )
    service = RobotBrokerService(
        executor=executor, token="secret", profile=profile,
        event_path=tmp_path / "events.jsonl",
    )
    denied = service.dispatch(
        "POST", "/v1/joint", {"joint": "shoulder_pan", "delta_deg": 0.1},
    )
    assert denied.status == 403
    assert executor._session is None


def test_simulated_http_stop_interrupts_active_native_motion_and_preserves_evidence(
    tmp_path,
) -> None:
    """Real HTTP concurrency with a fake arm; no Feetech hardware or physical safety claim."""
    started = threading.Event()
    cancelled = threading.Event()
    config = SOARM101Config(robot_id="so101")
    auth = AgentAuthorityStore(tmp_path / "human-lease.json")
    auth.issue(robot_id="so101", calibration_id="simulation", minutes=1)

    class FakeArm:
        calibration_id = None

        def __init__(self):
            self.config = config
            self.torque_enabled = False
            self.enabled_calls = 0

        def connect(self):
            pass

        def disconnect(self):
            pass

        def get_joint_positions(self):
            return SimpleNamespace(positions={"shoulder_pan": 0.0})

        def get_state(self):
            return SimpleNamespace(torque_enabled=self.torque_enabled)

        def enable(self):
            self.enabled_calls += 1
            self.torque_enabled = True

        def move_joints(self, positions, *, relative, speed, acceleration):
            assert relative and list(positions) == ["shoulder_pan"]
            started.set()
            if not cancelled.wait(timeout=4):
                raise RuntimeError("STOP did not interrupt active movement")
            return SimpleNamespace(
                accepted=True, completed=False, message="cancelled by STOP"
            )

        def hold(self):
            pass

        def stop(self):
            cancelled.set()

    arm = FakeArm()
    executor = SDKAgentExecutor(
        config=config, rates=AgentMotionRates(), simulation=True,
        authority_store=auth, arm_factory=lambda: arm,
    )
    profile = CapabilityProfile.from_document({
        "schema_version": 1, "name": "interruptible-fake",
        "tools": ["robot_health", "robot_state", "jog_joint", "stop"],
        "cameras": [], "limits": {"max_joint_delta_deg": 0.5},
    })
    events_path = tmp_path / "events.jsonl"
    service = RobotBrokerService(
        executor=executor, token="local-token", profile=profile,
        allow_simulated_sdk_posts=True, event_path=events_path,
    )
    server = RobotBrokerHTTPServer(("127.0.0.1", 0), service)
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()
    root = f"http://127.0.0.1:{server.server_address[1]}"
    joint_reply = []

    def post(path, payload, *, authorized=True):
        headers = {"Content-Type": "application/json"}
        if authorized:
            headers["Authorization"] = "Bearer local-token"
        request = urllib.request.Request(
            root + path,
            data=json.dumps(payload).encode("utf-8"),
            headers=headers,
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=4) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read())

    def joint_worker():
        try:
            joint_reply.append(post(
                "/v1/joint", {"joint": "shoulder_pan", "delta_deg": 0.1},
            ))
        except Exception as exc:
            joint_reply.append(exc)

    moving = threading.Thread(target=joint_worker)
    try:
        moving.start()
        assert started.wait(timeout=3)
        # Authentication and pinned profile checks still precede out-of-band STOP.
        code, body = post("/v1/stop", {}, authorized=False)
        assert code == 401 and body["ok"] is False
        assert not cancelled.is_set()
        code, body = post(
            "/v1/joint", {"joint": "shoulder_pan", "delta_deg": 1.0},
        )
        assert code == 403 and body["ok"] is False
        assert not cancelled.is_set()
        code, body = post("/v1/stop", {})
        assert code == 200 and body["ok"] is True
        assert body["result"]["holding"] is True
        assert cancelled.is_set()
        moving.join(timeout=3)
        assert not moving.is_alive()
        assert len(joint_reply) == 1
        assert joint_reply[0][0] == 409
        assert joint_reply[0][1]["ok"] is False
        assert arm.enabled_calls == 1
        events = [
            json.loads(row) for row in events_path.read_text().splitlines()
        ]
        statuses = {(row["action"], row["ok"]) for row in events}
        assert ("joint", False) in statuses
        assert ("stop", True) in statuses
        assert ("profile_rejection", False) in statuses
        assert len({row["profile_sha256"] for row in events}) == 1
        assert events[0]["profile_sha256"] == profile.public()["sha256"]
        assert all(row["request_id"] for row in events)
    finally:
        cancelled.set()
        moving.join(timeout=3)
        server.shutdown()
        server.server_close()
        http_thread.join(timeout=3)
        executor.close()


def test_simulated_native_stop_failure_is_audited_and_never_claims_completion(
    native, tmp_path, monkeypatch,
) -> None:
    executor, _ = native
    profile = CapabilityProfile.from_document({
        "schema_version": 1, "name": "stop-failure",
        "tools": ["robot_state", "stop"],
        "cameras": [], "limits": {},
    })
    service = RobotBrokerService(
        executor=executor, token="secret", profile=profile,
        allow_simulated_sdk_posts=True,
        event_path=tmp_path / "events.jsonl",
    )
    service.dispatch("GET", "/v1/state")

    def fail_hold(*args, **kwargs):
        raise RuntimeError("HOLD failed")

    monkeypatch.setattr(executor._session, "stop", fail_hold)
    result = service.dispatch("POST", "/v1/stop", {})
    assert result.status == 409
    assert result.body["ok"] is False
    assert "HOLD failed" in result.body["error"]
    entries = [
        json.loads(row)
        for row in (tmp_path / "events.jsonl").read_text().splitlines()
    ]
    assert entries[-1]["action"] == "stop"
    assert entries[-1]["ok"] is False
    assert entries[-1]["result"] == {}



def test_native_stop_cancels_queued_action_and_revokes_authority(
    native, tmp_path,
) -> None:
    """Deterministically queue a request after admission but before dispatch."""
    executor, auth = native
    profile = CapabilityProfile.from_document({
        "schema_version": 1, "name": "queued-stop",
        "tools": ["robot_state", "jog_joint", "stop"],
        "cameras": [], "limits": {"max_joint_delta_deg": 2.0},
    })
    service = RobotBrokerService(
        executor=executor, token="token", profile=profile,
        allow_simulated_sdk_posts=True,
        event_path=tmp_path / "events.jsonl",
    )
    auth.issue(robot_id="so101", calibration_id="simulation", minutes=1)
    assert service.dispatch("GET", "/v1/state").status == 200
    entered_queue = threading.Event()
    original = service._operation_lock

    class GateLock:
        def __enter__(self):
            entered_queue.set()
            original.acquire()
            return self

        def __exit__(self, *_args):
            original.release()

    service._operation_lock = GateLock()
    queued = []
    original.acquire()

    def run_queued():
        queued.append(service.dispatch(
            "POST", "/v1/joint",
            {"joint": "shoulder_pan", "delta_deg": 0.5},
        ))

    worker = threading.Thread(target=run_queued)
    worker.start()
    try:
        assert entered_queue.wait(timeout=2)
        # The queued request has captured the earlier STOP generation.
        stopped = service.dispatch("POST", "/v1/stop", {})
        assert stopped.status == 200
        assert stopped.body["result"]["holding"] is False
    finally:
        original.release()
        worker.join(timeout=3)

    assert not worker.is_alive()
    assert len(queued) == 1
    assert queued[0].status == 409
    assert "cancelled by a newer STOP" in queued[0].body["error"]
    assert auth.status()["armed"] is False
    assert executor._session.get_state().torque_enabled is False
    # A newly admitted request after STOP must also require a fresh lease.
    later = service.dispatch(
        "POST", "/v1/joint",
        {"joint": "shoulder_pan", "delta_deg": 0.5},
    )
    assert later.status == 409
    assert "not armed" in later.body["error"]


def test_native_stop_failure_also_revokes_lease(native, tmp_path, monkeypatch) -> None:
    executor, auth = native
    profile = CapabilityProfile.from_document({
        "schema_version": 1, "name": "failed-stop",
        "tools": ["robot_state", "stop"],
        "cameras": [], "limits": {},
    })
    service = RobotBrokerService(
        executor=executor, token="token", profile=profile,
        allow_simulated_sdk_posts=True,
        event_path=tmp_path / "events.jsonl",
    )
    auth.issue(robot_id="so101", calibration_id="simulation", minutes=1)
    assert service.dispatch("GET", "/v1/state").status == 200

    def failed_hold():
        raise RuntimeError("STOP/HOLD bus failure")

    monkeypatch.setattr(executor._session, "stop", failed_hold)
    response = service.dispatch("POST", "/v1/stop", {})
    assert response.status == 409
    assert "STOP/HOLD bus failure" in response.body["error"]
    assert auth.status()["armed"] is False



def test_native_camera_evidence_preserves_bytes_hash_and_redacts_host_path(
    native, tmp_path, monkeypatch,
) -> None:
    """No camera device opens: fake trusted SDK capture returns a local test JPEG."""
    executor, _ = native
    bytes_on_disk = b"\xff\xd8\xffsynthetic-camera-evidence\xff\xd9"
    picture = tmp_path / "private-capture.jpg"
    picture.write_bytes(bytes_on_disk)
    profile = CapabilityProfile.from_document({
        "schema_version": 1, "name": "fake-capture",
        "tools": ["capture_camera"],
        "cameras": ["overhead"], "limits": {},
    })
    service = RobotBrokerService(
        executor=executor, token="secret", profile=profile,
        allow_simulated_sdk_posts=True,
        event_path=tmp_path / "audit.jsonl",
    )
    original = executor.execute

    def fake_capture(action, request):
        if action == "capture":
            assert request == {"camera": "overhead"}
            return {
                "name": "overhead", "path": str(picture),
                "device": "/dev/video-test-private",
                "width": 12, "height": 8, "timestamp": 1.0,
            }
        return original(action, request)

    monkeypatch.setattr(executor, "execute", fake_capture)
    response = service.dispatch(
        "POST", "/v1/capture", {"camera": "overhead"},
    )
    assert response.status == 200
    payload = response.body["result"]
    expected_sha = hashlib.sha256(bytes_on_disk).hexdigest()
    assert payload["sha256"] == expected_sha
    assert base64.b64decode(payload["image_base64"]) == bytes_on_disk
    assert "path" not in payload and "device" not in payload
    assert str(picture) not in json.dumps(response.body)

    events = [
        json.loads(line)
        for line in (tmp_path / "audit.jsonl").read_text().splitlines()
    ]
    assert [row["action"] for row in events] == ["capture", "capture_evidence"]
    assert events[0]["result"]["sha256"] == expected_sha
    assert events[1]["result"]["host_path"] == str(picture)
    assert events[0]["request_id"] == events[1]["request_id"]
    assert all(
        row["profile_sha256"] == profile.public()["sha256"] for row in events
    )


def test_native_camera_missing_evidence_fails_before_success_event(
    native, tmp_path, monkeypatch,
) -> None:
    executor, _ = native
    profile = CapabilityProfile.from_document({
        "schema_version": 1, "name": "fake-capture-missing",
        "tools": ["capture_camera"], "cameras": ["overhead"], "limits": {},
    })
    service = RobotBrokerService(
        executor=executor, token="secret", profile=profile,
        allow_simulated_sdk_posts=True,
        event_path=tmp_path / "audit.jsonl",
    )

    def fake_missing_capture(action, request):
        assert action == "capture"
        return {"name": "overhead", "path": str(tmp_path / "missing.jpg")}

    monkeypatch.setattr(executor, "execute", fake_missing_capture)
    response = service.dispatch(
        "POST", "/v1/capture", {"camera": "overhead"},
    )
    assert response.status == 409
    assert response.body["ok"] is False
    events = [
        json.loads(line)
        for line in (tmp_path / "audit.jsonl").read_text().splitlines()
    ]
    assert len(events) == 1
    assert events[0]["action"] == "capture"
    assert events[0]["ok"] is False


@pytest.mark.parametrize("action,payload", [
    ("go_pose", {"name": "agent_start_overhead", "speed_deg_s": 16}),
    ("sleep", {"acceleration_deg_s2": 151}),
    ("sleep_up", {"gripper_speed_raw": 251}),
    ("joint", {"joint": "shoulder_pan", "delta_deg": 1, "speed_deg_s": 16}),
    ("jog", {"x_mm": 1, "speed_mm_s": 11}),
    ("gripper", {"target": "open", "gripper_acceleration_raw": 21}),
])
def test_direct_sdk_preview_rejects_faster_requests_before_device(
    tmp_path, action, payload,
) -> None:
    executor = SDKAgentExecutor(
        config=SOARM101Config(robot_id="so101"),
        rates=AgentMotionRates(joint_speed_deg_s=15, joint_acceleration_deg_s2=150),
        simulation=True,
        authority_store=AgentAuthorityStore(tmp_path / "lease.json"),
    )
    with pytest.raises(ValueError):
        executor.execute(action, payload)
    assert executor._session is None


def test_physical_trial_is_explicitly_opted_in_and_strictly_profiled(tmp_path) -> None:
    from soarm101_motion import SOARM101
    config = SOARM101Config(port="/dev/fake-tty", robot_id="so101")
    sdk = SDKAgentExecutor(
        config=config, rates=AgentMotionRates(),
        simulation=False, physical_trial=True,
        arm_factory=lambda: SOARM101.simulated(config=config),
    )
    full = CapabilityProfile.full()
    narrow = CapabilityProfile.from_document({
        "schema_version": 1, "name": "trial",
        "tools": ["robot_health", "robot_capabilities", "robot_state",
                  "go_pose", "sleep", "stop"],
        "cameras": [], "limits": {},
    })
    try:
        with pytest.raises(ValueError, match="profile"):
            RobotBrokerService(executor=sdk, token="token",
                               profile=full, allow_physical_sdk_trial=True)
        with pytest.raises(ValueError, match="loopback"):
            RobotBrokerHTTPServer(
                ("0.0.0.0", 0),
                RobotBrokerService(executor=sdk, token="token",
                                   profile=narrow, allow_physical_sdk_trial=True),
            )
        service = RobotBrokerService(executor=sdk, token="token", profile=narrow)
        assert service.dispatch("POST", "/v1/sleep", {"joint_only": True}).status == 403
    finally:
        sdk.close()


def test_loopback_trial_sleep_http_reuses_one_sdk_connection_without_cli(
    tmp_path, monkeypatch,
) -> None:
    from soarm101_motion import SOARM101
    import urllib.request
    from soarm101_motion.broker import AgentCommandExecutor

    config = SOARM101Config.from_motion_limits(
        robot_id="so101", port="/dev/test-fake-tty",
        default_joint_speed=0.10, default_joint_acceleration=0.50,
        max_joint_speed_deg_s=15, max_joint_acceleration_deg_s2=150,
    )
    authority = AgentAuthorityStore(tmp_path / "lease.json")
    authority.issue(robot_id="so101", calibration_id="simulation", minutes=5)
    sdk = SDKAgentExecutor(
        config=config,
        rates=AgentMotionRates(joint_speed_deg_s=15,
                               joint_acceleration_deg_s2=150),
        simulation=False, physical_trial=True,
        arm_factory=lambda: SOARM101.simulated(config=config),
        authority_store=authority,
    )
    # Test uses the real simulation backend behind an explicitly physical
    # *trial service*. It never opens the requested fake tty device.
    monkeypatch.setattr(sdk, "_calibration_id", lambda arm: "simulation")
    from soarm101_motion.sdk_capabilities import SDK_CAPABILITIES
    import soarm101_motion.poses as poses
    monkeypatch.setattr(
        poses, "default_pose_library_path",
        lambda robot_id: tmp_path / f"{robot_id}-trial-poses.json",
    )
    for pose_name in ("agent_start_overhead",
                      "agent_start_overhead_left", "agent_start_overhead_right"):
        SDK_CAPABILITIES.dispatch(
            "capture_saved_pose", sdk._arm(),
            {"robot_id": "so101", "name": pose_name, "source": "follower"},
        )
    # The real operator arms through a separate CLI process before the
    # broker opens; simulate that already-active torque HOLD here.
    sdk._arm().enable()
    monkeypatch.setattr(
        AgentCommandExecutor, "run",
        lambda *a, **k: pytest.fail("physical persistent broker invoked CLI"),
    )
    profile = CapabilityProfile.from_document({
        "schema_version": 1, "name": "trial",
        "tools": ["robot_health", "robot_capabilities", "robot_state",
                  "go_pose", "sleep", "stop"],
        "cameras": [], "limits": {},
    })
    service = RobotBrokerService(
        executor=sdk, token="test-token",
        event_path=tmp_path / "events.jsonl",
        profile=profile, allow_physical_sdk_trial=True,
    )
    server = RobotBrokerHTTPServer(("127.0.0.1", 0), service)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    def request(method, route, payload=None):
        data = json.dumps(payload or {}).encode() if method == "POST" else None
        req = urllib.request.Request(
            f"http://127.0.0.1:{server.server_port}{route}",
            data=data, method=method,
            headers={"Authorization": "Bearer test-token"},
        )
        with urllib.request.urlopen(req, timeout=30) as response:
            return json.load(response)

    try:
        first = request("GET", "/v1/state")["result"]
        assert first["broker_session"]["sdk_connect_count"] == 1
        assert first["broker_session"]["mode"] == "physical_trial"
        results = []
        for route, payload in (
            ("/v1/sleep", {"joint_only": True}),
            ("/v1/go-pose", {"name": "agent_start_overhead", "joint_only": True}),
            ("/v1/go-pose", {"name": "agent_start_overhead_left", "joint_only": True}),
            ("/v1/go-pose", {"name": "agent_start_overhead_right", "joint_only": True}),
            ("/v1/sleep", {"joint_only": True}),
        ):
            reply = request("POST", route, payload)["result"]
            results.append(reply)
            assert reply["completed"] and reply["accepted"]
            assert reply["broker_session"] == first["broker_session"]
        assert [item["action"] for item in results] == [
            "sleep", "go_pose", "go_pose", "go_pose", "sleep",
        ]
        assert sdk.connection_count == 1
        stop = request("POST", "/v1/stop", {})["result"]
        assert stop["completed"]
        assert authority.status()["armed"] is False
    finally:
        server.shutdown()
        server.server_close()
        sdk.close()


def test_physical_trial_rejects_invalid_joint_only_types_before_connect(tmp_path) -> None:
    config = SOARM101Config(port="/dev/fake-tty", robot_id="so101")
    sdk = SDKAgentExecutor(
        config=config, rates=AgentMotionRates(), physical_trial=True,
        arm_factory=lambda: pytest.fail("should not connect"),
    )
    try:
        with pytest.raises(ValueError, match="boolean"):
            sdk.execute("sleep", {"joint_only": 1})
        with pytest.raises(ValueError, match="unknown"):
            sdk.execute("go_pose", {"name": "agent_test", "host_path": "/tmp"})
        assert sdk.connection_count == 0
    finally:
        sdk.close()


def test_physical_broker_trace_requires_operator_tty_and_has_no_yes_bypass(
    monkeypatch, tmp_path,
) -> None:
    """No connected robot, no broker, and no output file from unattended launch."""
    import importlib.util
    import io
    from pathlib import Path

    script = Path(__file__).resolve().parents[1] / "examples" / "persistent_broker_motion_trace.py"
    spec = importlib.util.spec_from_file_location("physical_broker_trace_test", script)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    command = ["--port", "/dev/ttyACM1", "--output", str(tmp_path / "trace.jsonl")]
    with pytest.raises(SystemExit):
        module.build_parser().parse_args([*command, "--yes"])
    monkeypatch.setattr(module.sys, "stdin", io.StringIO(""))
    with pytest.raises(PermissionError, match="interactive operator terminal"):
        module.main(command)
    assert not list(tmp_path.iterdir())


def test_supervised_physical_agent_profile_is_exact_and_low_delta(
    tmp_path,
) -> None:
    """Hardware-free physical-session policy gate; no serial/servo access."""
    from soarm101_motion import SOARM101

    config = SOARM101Config(
        port="/dev/fake", robot_id="so101", auto_enable_torque=False,
    )
    executor = SDKAgentExecutor(
        config=config, rates=AgentMotionRates(
            joint_speed_deg_s=2, joint_acceleration_deg_s2=10,
        ), physical_trial=True,
        arm_factory=lambda: SOARM101.simulated(config=config),
        authority_store=AgentAuthorityStore(tmp_path / "lease.json"),
    )
    def profile(delta, tools=None):
        return CapabilityProfile.from_document({
            "schema_version": 1, "name": "operator-attended",
            "tools": tools or [
                "robot_health", "robot_capabilities", "robot_state",
                "jog_joint", "stop",
            ], "cameras": [],
            "limits": {"max_joint_delta_deg": delta},
        })
    service = RobotBrokerService(
        executor=executor, token="private", profile=profile(1),
        allow_physical_sdk_trial=True,
    )
    assert service._physical_supervised_agent_trial is True
    with pytest.raises(ValueError, match="physical trial profile"):
        RobotBrokerService(
            executor=executor, token="private", profile=profile(2),
            allow_physical_sdk_trial=True,
        )
    with pytest.raises(ValueError, match="physical trial profile"):
        RobotBrokerService(
            executor=executor, token="private",
            profile=profile(1, tools=[
                "robot_health", "robot_capabilities", "robot_state",
                "jog_joint", "stop", "sleep",
            ]), allow_physical_sdk_trial=True,
        )
    executor.close()


def test_supervised_physical_jog_never_implicitly_rearms_torque(
    tmp_path, monkeypatch,
) -> None:
    """A fake physical SDK factory keeps real serial devices unopened."""
    from soarm101_motion import SOARM101
    from soarm101_motion.exceptions import HardwareFaultError

    config = SOARM101Config(
        port="/dev/fake", robot_id="so101", auto_enable_torque=False,
    )
    arm = SOARM101.simulated(config=config, realtime=False)
    arm.connect()
    arm.enable()
    authority = AgentAuthorityStore(tmp_path / "lease.json")
    authority.issue(robot_id="so101", calibration_id="simulation", minutes=5)
    executor = SDKAgentExecutor(
        config=config, rates=AgentMotionRates(
            joint_speed_deg_s=2, joint_acceleration_deg_s2=10,
        ), physical_trial=True, arm_factory=lambda: arm,
        authority_store=authority,
    )
    monkeypatch.setattr(
        executor, "_calibration_id", lambda current: "simulation",
    )
    monkeypatch.setattr(
        arm, "enable", lambda: pytest.fail("unexpected 1/1 servo rearm latch"),
    )
    try:
        motion = executor.execute(
            "joint", {"joint": "shoulder_pan", "delta_deg": 0.1},
        )
        assert motion["completed"] is True
        assert executor.connection_count == 1
        arm.tool.disable()
        # A torque-off target may not silently enable motor power.
        # The fake arm state remains usable for deterministic refusal.
        with pytest.raises((HardwareFaultError, RuntimeError)):
            executor.execute(
                "joint", {"joint": "shoulder_pan", "delta_deg": 0.1},
            )
    finally:
        executor.close()


def test_joint_only_physical_overhead_reuses_human_torque_hold(
    tmp_path, monkeypatch,
) -> None:
    """One-shot saved pose must not inject the old 1/1 enable latch."""
    from soarm101_motion import SOARM101
    from soarm101_motion.sdk_capabilities import SDK_CAPABILITIES
    import soarm101_motion.poses as poses

    config = SOARM101Config(port="/dev/fake-device", robot_id="so101")
    arm = SOARM101.simulated(config=config, realtime=False)
    arm.connect()
    arm.enable()
    monkeypatch.setattr(
        poses, "default_pose_library_path",
        lambda robot_id: tmp_path / f"{robot_id}-saved-poses.json",
    )
    SDK_CAPABILITIES.dispatch(
        "capture_saved_pose", arm,
        {"robot_id": "so101", "name": "agent_start_overhead",
         "source": "follower"},
    )
    monkeypatch.setattr(
        arm, "enable", lambda: pytest.fail("unexpected 1/1 torque relatch"),
    )
    authority = AgentAuthorityStore(tmp_path / "human-lease.json")
    authority.issue(robot_id="so101", calibration_id="simulation", minutes=5)
    executor = SDKAgentExecutor(
        config=config, rates=AgentMotionRates(),
        physical_trial=True, arm_factory=lambda: arm, authority_store=authority,
    )
    monkeypatch.setattr(executor, "_calibration_id", lambda current: "simulation")
    try:
        result = executor.execute(
            "go_pose", {"name": "agent_start_overhead", "joint_only": True},
        )
        assert result["accepted"] and result["completed"]
        assert result["joint_only"] is True
        assert executor.connection_count == 1
    finally:
        executor.close()


def test_joint_only_physical_overhead_rejects_disabled_torque_before_goal(
    tmp_path, monkeypatch,
) -> None:
    from soarm101_motion import SOARM101
    from soarm101_motion.sdk_capabilities import SDK_CAPABILITIES
    from soarm101_motion.exceptions import HardwareFaultError
    import soarm101_motion.poses as poses

    config = SOARM101Config(port="/dev/fake-device", robot_id="so101")
    arm = SOARM101.simulated(config=config, realtime=False)
    arm.connect()
    monkeypatch.setattr(
        poses, "default_pose_library_path",
        lambda robot_id: tmp_path / f"{robot_id}-saved-poses.json",
    )
    SDK_CAPABILITIES.dispatch(
        "capture_saved_pose", arm,
        {"robot_id": "so101", "name": "agent_start_overhead",
         "source": "follower"},
    )
    monkeypatch.setattr(
        arm, "enable", lambda: pytest.fail("torque-off must not rearm itself"),
    )
    authority = AgentAuthorityStore(tmp_path / "human-lease.json")
    authority.issue(robot_id="so101", calibration_id="simulation", minutes=5)
    executor = SDKAgentExecutor(
        config=config, rates=AgentMotionRates(),
        physical_trial=True, arm_factory=lambda: arm, authority_store=authority,
    )
    monkeypatch.setattr(executor, "_calibration_id", lambda current: "simulation")
    try:
        with pytest.raises(HardwareFaultError, match="stationary torque HOLD"):
            executor.execute(
                "go_pose", {"name": "agent_start_overhead", "joint_only": True},
            )
        assert authority.status()["armed"] is False
    finally:
        executor.close()


def test_joint_only_physical_sleep_reuses_existing_torque_hold_without_rearming(
    tmp_path, monkeypatch,
) -> None:
    """A broker reconnect must not issue a second 1/1 torque-enable latch."""
    from soarm101_motion import SOARM101
    config = SOARM101Config(port="/dev/fake-device", robot_id="so101")
    arm = SOARM101.simulated(config=config)
    arm.connect()
    arm.enable()
    assert arm.get_state().torque_enabled is True
    monkeypatch.setattr(
        arm, "enable",
        lambda: pytest.fail("redundant torque-enable during canonical Sleep"),
    )
    authority = AgentAuthorityStore(tmp_path / "human-lease.json")
    authority.issue(robot_id="so101", calibration_id="simulation", minutes=5)
    executor = SDKAgentExecutor(
        config=config, rates=AgentMotionRates(),
        simulation=False, physical_trial=True,
        arm_factory=lambda: arm, authority_store=authority,
    )
    monkeypatch.setattr(executor, "_calibration_id", lambda current: "simulation")
    try:
        result = executor.execute("sleep", {"joint_only": True})
        assert result["accepted"] is True
        assert result["completed"] is True
        assert result["action"] == "sleep"
        assert result["holding"] is True
        assert executor.connection_count == 1
    finally:
        executor.close()


def test_joint_only_physical_sleep_rejects_torque_off_without_enabling(
    tmp_path, monkeypatch,
) -> None:
    from soarm101_motion import SOARM101
    from soarm101_motion.exceptions import HardwareFaultError
    config = SOARM101Config(port="/dev/fake-device", robot_id="so101")
    arm = SOARM101.simulated(config=config)
    arm.connect()
    assert arm.get_state().torque_enabled is False
    monkeypatch.setattr(
        arm, "enable",
        lambda: pytest.fail("failed preflight must not enable motors"),
    )
    authority = AgentAuthorityStore(tmp_path / "human-lease.json")
    authority.issue(robot_id="so101", calibration_id="simulation", minutes=5)
    executor = SDKAgentExecutor(
        config=config, rates=AgentMotionRates(),
        simulation=False, physical_trial=True,
        arm_factory=lambda: arm, authority_store=authority,
    )
    monkeypatch.setattr(executor, "_calibration_id", lambda current: "simulation")
    try:
        with pytest.raises(HardwareFaultError, match="stationary torque HOLD"):
            executor.execute("sleep", {"joint_only": True})
        assert authority.status()["armed"] is False
    finally:
        executor.close()
