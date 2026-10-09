"""Simulation-only tests for the staged direct SDK broker executor."""

from __future__ import annotations

import threading
from types import SimpleNamespace

import pytest

from soarm101_motion.agent_control import AgentAuthorityStore
from soarm101_motion.broker import AgentMotionRates, RobotBrokerService, build_parser
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
