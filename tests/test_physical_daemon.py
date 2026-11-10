from __future__ import annotations

import json
import threading
import time
from types import SimpleNamespace

import pytest

from soarm101_motion.agent_control import AgentAuthorityStore
from soarm101_motion.config import SOARM101Config
from soarm101_motion.physical_daemon import (
    _heartbeat_loop, _load_config, _make_executor, _preflight,
)
import soarm101_motion.physical_daemon as physical_daemon


class FakeExecutor:
    def __init__(self, state, *, operator_choreography_only=False):
        self.operator_choreography_only = operator_choreography_only
        self.state = SimpleNamespace(**state)
        self.actions = []
        self._session = SimpleNamespace(
            config=SimpleNamespace(robot_id="arm-a"),
            calibration_id=state.get("calibration_id", "cal-a"),
            get_state=lambda: self.state,
        )

    def execute(self, action, request):
        self.actions.append(action)
        assert action == "state"
        # Match the real broker's read_pose response: identity is present, but
        # sensed connection/motion/torque fields live on arm.get_state().
        return {"robot_id": "arm-a", "calibration_id": "cal-a",
                "joint_positions_rad": {"shoulder": 0.0}}

    def _assert_session_identity(self, arm):
        assert arm is self._session

    @staticmethod
    def _calibration_id(arm):
        return arm.calibration_id


def _state():
    return {"robot_id": "arm-a", "calibration_id": "cal-a", "connected": True,
            "faulted": False, "moving": False, "torque_enabled": True}


def test_preflight_requires_lease_without_sdk_io(tmp_path):
    executor = FakeExecutor(_state())
    with pytest.raises(PermissionError, match="not armed"):
        _preflight(executor, "arm-a", "cal-a", AgentAuthorityStore(tmp_path / "lease.json"))
    assert executor.actions == []


def test_preflight_requires_exact_identity_and_sensed_hold(tmp_path):
    authority = AgentAuthorityStore(tmp_path / "lease.json")
    authority.issue(robot_id="arm-a", calibration_id="cal-a")
    executor = FakeExecutor(_state())
    assert _preflight(executor, "arm-a", "cal-a", authority)["moving"] is False
    assert executor.actions == ["state"]
    for key, value in (("calibration_id", "other"), ("moving", True),
                       ("torque_enabled", False), ("faulted", True)):
        bad = FakeExecutor({**_state(), key: value})
        with pytest.raises(PermissionError):
            _preflight(bad, "arm-a", "cal-a", authority)


def test_operator_preflight_needs_no_agent_lease_but_keeps_identity_and_hold_checks():
    executor = FakeExecutor(_state(), operator_choreography_only=True)
    result = _preflight(executor, "arm-a", "cal-a", None)
    assert result["moving"] is False and result["torque_enabled"] is True
    for key, value in (("calibration_id", "other"), ("moving", True),
                       ("torque_enabled", False)):
        bad = FakeExecutor({**_state(), key: value}, operator_choreography_only=True)
        with pytest.raises(PermissionError):
            _preflight(bad, "arm-a", "cal-a", None)


def test_startup_hold_preflight_allows_torque_off_only_after_identity_health_checks():
    executor = FakeExecutor({**_state(), "torque_enabled": False}, operator_choreography_only=True)
    assert _preflight(executor, "arm-a", "cal-a", None, require_hold=False)["torque_enabled"] is False
    for key, value in (("connected", False), ("faulted", True), ("moving", True), ("calibration_id", "other")):
        bad = FakeExecutor({**_state(), "torque_enabled": False, key: value}, operator_choreography_only=True)
        with pytest.raises(PermissionError):
            _preflight(bad, "arm-a", "cal-a", None, require_hold=False)


@pytest.mark.parametrize("marker", ["startup-hold-failure.json", "operator-reconciliation-required.json"])
def test_startup_hold_failure_marker_prevents_new_session_before_device_io(tmp_path, marker):
    config = _serve_config(tmp_path)
    state_dir = tmp_path / "state"
    state_dir.mkdir()
    (state_dir / marker).write_text('{"retry_safe": false}')
    with pytest.raises(RuntimeError, match="operator reconciliation"):
        physical_daemon.serve(config, executor_factory=lambda **kwargs: pytest.fail("must not reconnect"))


@pytest.mark.parametrize("value", ["true", 1, None])
def test_startup_hold_config_rejects_non_boolean(tmp_path, value):
    config = {**_serve_config(tmp_path), "physical": True, "operator_choreography_only": True,
              "establish_hold_on_startup": value}
    path = tmp_path / "daemon.json"
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match="must be boolean"):
        _load_config(path)


def test_daemon_establishes_hold_under_real_watchdog_before_serving(tmp_path, monkeypatch):
    from soarm101_motion.arm import SOARM101
    from soarm101_motion.broker_sdk import SDKAgentExecutor
    from soarm101_motion.choreography import ChoreographyPolicy, JointChoreography, JointFrame, JointTarget
    from soarm101_motion.constants import ARM_JOINTS
    import soarm101_motion.broker_sdk as broker_sdk

    config = {**_serve_config(tmp_path), "robot_id": "so101", "calibration_id": "simulation",
              "operator_choreography_only": True, "establish_hold_on_startup": True}
    program = JointChoreography.author("idle", "so101", "simulation", (
        JointFrame(0., (0.,) * 5), JointFrame(.3, (0.,) * 5)))
    target = JointTarget.author("wake_to_neutral", "so101", "simulation", (0.,) * 5)
    policy = ChoreographyPolicy("so101", "simulation", (program.revision, target.revision), allow_physical_trial=True)
    monkeypatch.setattr(physical_daemon, "load_trusted_choreography_catalog", lambda _: (
        policy, {"idle": program, "wake_to_neutral": target}))
    monkeypatch.setattr(broker_sdk, "STOP_STABILITY_SAMPLE_DELAYS_S", (0, 0))
    enabled = []
    executors = []
    def factory(**kwargs):
        # Deliberately inject the SDK simulation backend into the physical
        # supervisor's factory seam. This never opens the configured port.
        arm = SOARM101.simulated(config=kwargs["config"])
        arm.backend.calibration = SimpleNamespace(calibration_id="simulation", motors={})
        original_enable = arm.enable
        def enable():
            executor = executors[0]
            executor.motion_guard()  # real BrokerWatchdog is active first
            enabled.append(dict(arm.get_joint_positions().positions))
            original_enable()
        arm.enable = enable
        executor = SDKAgentExecutor(**kwargs, arm_factory=lambda: arm)
        executors.append(executor)
        return executor
    class Server:
        def __init__(self, address, service):
            self.service = service
        def serve_forever(self, **kwargs):
            executor = executors[0]
            assert executor._startup_hold_verified
            assert executor.config.joint_position_tolerance_rad <= policy.max_start_error_rad
            assert len(enabled) == 1  # the startup itself latches only once
            assert executor._session.get_state().torque_enabled
            assert self.service.watchdog.status["state"] == "watching"
            assert set(enabled[0]) == set(ARM_JOINTS)
            assert json.loads((tmp_path / "state/startup-hold.json").read_text())["holding"]
            # Exercise the actual operator-only trusted routes without a
            # human agent lease, including the independently admitted STOP.
            wake = self.service.dispatch_trusted_host("POST", "/v1/trusted/target", {
                "request_id": "startup-wake", "name": target.name, "revision": target.revision})
            assert wake.status == 200, wake.body
            idle = self.service.dispatch_trusted_host("POST", "/v1/trusted/choreography", {
                "request_id": "startup-idle", "name": program.name, "revision": program.revision})
            assert idle.status == 200, idle.body
            assert self.service.dispatch("POST", "/v1/joint", {"joint": "wrist_roll", "delta_deg": 1}).status == 403
            stopped = self.service.dispatch("POST", "/v1/stop", {})
            assert stopped.status == 200, stopped.body
            assert stopped.body["result"]["holding"] is True
        def server_close(self):
            pass
    monkeypatch.setattr(physical_daemon, "RobotBrokerHTTPServer", Server)
    monkeypatch.setattr(physical_daemon.signal, "signal", lambda *args: None)
    assert physical_daemon.serve(config, executor_factory=factory) == 0
    assert not (tmp_path / "state/startup-hold-failure.json").exists()


def test_preflight_rejects_connection_failure_without_stop(tmp_path):
    authority = AgentAuthorityStore(tmp_path / "lease.json")
    authority.issue(robot_id="arm-a", calibration_id="cal-a")
    executor = FakeExecutor({**_state(), "connected": False})
    with pytest.raises(PermissionError, match="healthy stationary"):
        _preflight(executor, "arm-a", "cal-a", authority)
    assert executor.actions == ["state"]


def _serve_config(tmp_path):
    lease = tmp_path / "lease.json"
    AgentAuthorityStore(lease).issue(robot_id="arm-a", calibration_id="cal-a")
    return {"robot_id": "arm-a", "port": "/dev/unused", "calibration_id": "cal-a",
            "lease_path": str(lease), "operator_choreography_only": False,
            "trusted_catalog_path": str(tmp_path / "catalog.json"),
            "loopback_port": 18765, "state_dir": str(tmp_path / "state")}


def test_catalog_failure_cleans_only_owned_tokens_and_unlocks(tmp_path, monkeypatch):
    config = _serve_config(tmp_path)
    def fail_catalog(_path):
        raise ValueError("bad catalog")
    monkeypatch.setattr(physical_daemon, "load_trusted_choreography_catalog", fail_catalog)
    with pytest.raises(ValueError, match="bad catalog"):
        physical_daemon.serve(config)
    state_dir = tmp_path / "state"
    assert not (state_dir / "broker.token").exists()
    assert not (state_dir / "trusted-host.token").exists()
    lock_fd = physical_daemon.os.open(state_dir / "daemon.lock", physical_daemon.os.O_RDWR)
    try:
        physical_daemon.fcntl.flock(lock_fd, physical_daemon.fcntl.LOCK_EX | physical_daemon.fcntl.LOCK_NB)
    finally:
        physical_daemon.os.close(lock_fd)


def test_preexisting_token_is_never_removed_on_acquisition_failure(tmp_path, monkeypatch):
    config = _serve_config(tmp_path)
    state_dir = tmp_path / "state"
    state_dir.mkdir()
    existing = state_dir / "broker.token"
    existing.write_text("owned elsewhere")
    with pytest.raises(FileExistsError):
        physical_daemon.serve(config)
    assert existing.read_text() == "owned elsewhere"
    assert not (state_dir / "trusted-host.token").exists()


def test_physical_config_requires_explicit_marker_and_exact_fields(tmp_path):
    config = {"physical": True, "robot_id": "arm-a", "port": "/dev/ttyUSB9",
              "calibration_id": "cal-a", "lease_path": str(tmp_path / "lease.json"),
              "operator_choreography_only": False,
              "trusted_catalog_path": str(tmp_path / "catalog.json"),
              "loopback_port": 18765, "state_dir": str(tmp_path / "state")}
    path = tmp_path / "daemon.json"
    path.write_text(json.dumps(config))
    assert _load_config(path)["physical"] is True
    config["physical"] = False
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match="physical: true"):
        _load_config(path)


def test_physical_sdk_config_never_changes_torque_policy():
    config = SOARM101Config(robot_id="arm-a", port="/dev/ttyUSB9",
                            auto_enable_torque=False,
                            configure_motors_on_connect=False,
                            disable_torque_on_disconnect=False)
    assert config.auto_enable_torque is False
    assert config.configure_motors_on_connect is False
    assert config.disable_torque_on_disconnect is False


def test_executor_factory_receives_explicit_physical_mode_without_torque_changes(tmp_path):
    authority = AgentAuthorityStore(tmp_path / "lease.json")
    captured = {}
    def factory(**kwargs):
        captured.update(kwargs)
        return object()
    config = {"robot_id": "arm-a", "port": "/dev/ttyUSB9",
              "operator_choreography_only": False}
    _make_executor(config, authority=authority, executor_factory=factory)
    sdk_config = captured["config"]
    assert captured["physical_actions"] is True
    assert captured["operator_choreography_only"] is False
    assert captured["authority_store"] is authority
    assert sdk_config.auto_enable_torque is False
    assert sdk_config.configure_motors_on_connect is False
    assert sdk_config.disable_torque_on_disconnect is False


def test_operator_selected_medium_rates_reach_the_existing_executor(tmp_path):
    captured = {}
    def factory(**kwargs):
        captured.update(kwargs)
        return object()
    _make_executor({"robot_id": "arm", "port": "/dev/unused", "operator_choreography_only": True,
        "joint_speed_deg_s": 50.0, "joint_acceleration_deg_s2": 500.0},
        authority=None, executor_factory=factory)
    assert captured["rates"].joint_speed_deg_s == 50.0
    assert captured["rates"].joint_acceleration_deg_s2 == 500.0


def test_supervisor_heartbeat_is_host_owned_and_stops_cleanly():
    class Watchdog:
        count = 0
        def heartbeat(self):
            self.count += 1

    watchdog = Watchdog()
    stopped = threading.Event()
    worker = threading.Thread(target=_heartbeat_loop, args=(watchdog, stopped))
    worker.start()
    time.sleep(0.65)
    stopped.set()
    worker.join(timeout=1)
    assert worker.is_alive() is False
    assert watchdog.count >= 1


def test_shutdown_stops_owned_inflight_work_before_close_waits(monkeypatch):
    events = []
    lock = threading.Lock()
    lock.acquire()  # Represents a worker holding SDKAgentExecutor._dispatch_lock.

    class Executor:
        _motion_action_attempted = True
        def _fault(self, _error):
            events.append("fenced")
        def execute(self, action, _request):
            assert action == "stop"
            assert lock.locked()  # STOP did not wait for the dispatch lock.
            events.append("stop")
            return {"completed": True, "holding": True}
        def close(self):
            events.append("close_wait")
            assert not lock.locked()

    class Watchdog:
        status = {"stop_result": None, "stop_error": None}
        stopped = False
        def begin_stop(self):
            if self.stopped:
                raise PermissionError("already attempted")
            self.stopped = True
        def finish_stop(self, result=None, error=None):
            self.status = {"stop_result": result, "stop_error": error}

    executor = Executor()
    watchdog = Watchdog()
    service = SimpleNamespace(watchdog=watchdog)
    # Exercise the same ordering used by serve's finally block.
    physical_daemon._shutdown_supervisor(service, executor)
    lock.release()
    executor.close()
    assert events == ["fenced", "stop", "close_wait"]


def test_shutdown_does_not_stop_without_owned_dispatch():
    class Executor:
        _motion_action_attempted = False
        def _fault(self, _error):
            pytest.fail("startup/preflight failure must not be treated as owned motion")

    physical_daemon._shutdown_supervisor(SimpleNamespace(watchdog=None), Executor())


@pytest.mark.parametrize("previous", [
    {"completed": True, "holding": True},
    None,
])
def test_shutdown_does_not_repeat_prior_stop_and_surfaces_unknown(monkeypatch, previous):
    from soarm101_motion import simulated_replay

    calls = []
    class Executor:
        _motion_action_attempted = True
        def _fault(self, _error):
            calls.append("fenced")
        def execute(self, *_args):
            calls.append("stop")
            return {"completed": True, "holding": True}
    class Watchdog:
        status = {"stop_result": previous, "stop_error": "lost response" if previous is None else None}
        def begin_stop(self):
            raise PermissionError("STOP already attempted")

    monkeypatch.setattr(simulated_replay, "_stop_after_failure", lambda _e, _w: (
        {"outcome": "confirmed", "result": previous} if previous else
        {"outcome": "unconfirmed", "detail": "lost response"}
    ))
    if previous is None:
        with pytest.raises(RuntimeError, match="unconfirmed"):
            physical_daemon._shutdown_supervisor(SimpleNamespace(watchdog=Watchdog()), Executor())
    else:
        physical_daemon._shutdown_supervisor(SimpleNamespace(watchdog=Watchdog()), Executor())
    assert calls == ["fenced"]
