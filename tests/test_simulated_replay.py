"""Simulation-only recording replay checks. No device or physical arm is opened."""
import numpy as np
import pytest
import threading
from dataclasses import replace

from soarm101_motion.agent_control import AgentAuthorityStore
from soarm101_motion.broker import AgentMotionRates, RobotBrokerService
from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.capability_profile import CapabilityProfile
from soarm101_motion.config import SOARM101Config
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.replay_policy import ReplayPolicy
from soarm101_motion.replay_snapshot import load_pinned_recording
from soarm101_motion.simulated_replay import ReplayExecutionError, replay_simulated_recording
from soarm101_motion.trajectories import Trajectory, TrajectoryLibrary
from soarm101_motion.types import MotionResult
from soarm101_motion.asset_catalog import _file_digest, _revision


@pytest.fixture
def simulation(tmp_path):
    auth = AgentAuthorityStore(tmp_path / "lease.json")
    executor = SDKAgentExecutor(config=SOARM101Config(robot_id="so101"), rates=AgentMotionRates(),
                                simulation=True, simulation_actions=True, authority_store=auth)
    service = RobotBrokerService(executor=executor, token="fake", event_path=tmp_path / "events",
                                 allow_simulated_sdk_posts=True, watchdog_timeout_s=2)
    watchdog = service.watchdog
    assert watchdog is not None
    watchdog.start()
    assert service.dispatch("GET", "/v1/state").status == 200
    arm = executor._session
    start = arm.get_joint_positions().positions
    gripper = arm.tool.get_position()
    library = TrajectoryLibrary("so101", root=tmp_path / "recordings")
    samples = [start[name] for name in ARM_JOINTS]
    entry = library.save("still", Trajectory(
        timestamps_s=np.array([0.0, 0.1]), joints_rad=np.array([samples, samples]),
        gripper=np.array([gripper, gripper]),
        metadata={"source_robot_id": "so101", "source_calibration_id": "simulation"},
    ), kind="raw")
    revision = _revision({"data_sha256": _file_digest(entry.data_path),
                          "metadata_sha256": _file_digest(entry.metadata_path)})
    ident = "soarm101/recording/raw/still"
    policy = ReplayPolicy.from_document({"schema_version": 1, "robot_id": "so101",
        "calibration_id": "simulation", "allowed_assets": {ident: revision},
        "max_speed_scale": 1.0, "max_steps": 10, "max_duration_s": 5.0,
        "max_start_error_rad": 0.05, "max_start_gripper_error": 0.05})
    snapshot = load_pinned_recording(policy, ident, library)
    try:
        yield executor, watchdog, auth, policy, snapshot
    finally:
        watchdog.close()
        executor.close()


def test_simulated_recording_requires_lease_and_supervisor(simulation):
    executor, watchdog, auth, policy, snapshot = simulation
    with pytest.raises(PermissionError, match="heartbeat"):
        replay_simulated_recording(executor, watchdog, policy, snapshot, speed_scale=1.0)
    auth.issue(robot_id="so101", calibration_id="simulation", minutes=1)
    watchdog.heartbeat()
    result = replay_simulated_recording(executor, watchdog, policy, snapshot, speed_scale=1.0)
    assert result["mode"] == "simulation"
    assert result["completion"]["completed"] is True
    assert result["revision"] == snapshot.revision


def test_restart_quarantines_durable_replay_intent_until_confirmed_stop(simulation, tmp_path):
    executor, watchdog, _auth, policy, snapshot = simulation
    event_path = tmp_path / "events"
    request = {
        "request_id": "replay-before-crash", "session_id": executor.session_id,
        "asset_id": snapshot.asset_id, "revision": snapshot.revision,
        "speed_scale": 1.0,
    }
    poses = tmp_path / "poses.json"
    sequences = tmp_path / "sequences.json"
    poses.write_text("{}", encoding="utf-8")
    sequences.write_text("{}", encoding="utf-8")
    recordings = tmp_path / "recordings"
    recordings.mkdir(exist_ok=True)
    catalog_paths = (poses, recordings, sequences)
    profile = CapabilityProfile.full()
    profile = replace(profile, allowed_tools=profile.allowed_tools | {
        "list_assets", "replay_recording", "stop",
    })
    first = RobotBrokerService(
        executor=executor, token="fake", event_path=event_path,
        allow_simulated_sdk_posts=True, watchdog_timeout_s=2,
        replay_policy=policy, catalog_paths=catalog_paths, profile=profile,
    )
    # Model process death after the durable admission record but before outcome.
    first._reserve_replay_identity("http-request", request)
    restarted = RobotBrokerService(
        executor=executor, token="fake", event_path=event_path,
        allow_simulated_sdk_posts=True, watchdog_timeout_s=2,
        replay_policy=policy, catalog_paths=catalog_paths, profile=profile,
    )
    assert restarted._replay_quarantined
    response = restarted._dispatch_replay("new-http-request", {
        **request, "request_id": "new-replay-id",
    })
    assert response.status == 409
    assert "confirmed STOP required" in response.body["error"]

    # Only explicit positive holding evidence releases the durable quarantine.
    restarted._reconcile_replay_stop("stop-request", {"request_id": "stop"})
    recovered = RobotBrokerService(
        executor=executor, token="fake", event_path=event_path,
        allow_simulated_sdk_posts=True, watchdog_timeout_s=2,
        replay_policy=policy, catalog_paths=catalog_paths, profile=profile,
    )
    assert not recovered._replay_quarantined
    # Confirmed STOP clears the global quarantine but never frees a durable
    # caller ID for replay after process restart.
    reused = recovered._dispatch_replay("new-http-request", {
        **request, "request_id": "replay-before-crash",
    })
    assert reused.status == 409
    assert "unknown after broker restart" in reused.body["error"]
    changed_payload = recovered._dispatch_replay("another-http-request", {
        **request, "request_id": "replay-before-crash", "speed_scale": 0.5,
    })
    assert changed_payload.status == 409
    assert "bound to another payload" in changed_payload.body["error"]


@pytest.mark.parametrize("fragment", [
    b'{"action":"replay_identity_reserved","request":',
    b"\xfftruncated-journal-record",
])
def test_truncated_replay_journal_stays_quarantined_until_stop_and_keeps_ids(
    simulation, tmp_path, fragment,
):
    executor, _watchdog, _auth, policy, snapshot = simulation
    event_path = tmp_path / "events"
    event_path.write_bytes(fragment)
    poses = tmp_path / "poses.json"
    sequences = tmp_path / "sequences.json"
    poses.write_text("{}", encoding="utf-8")
    sequences.write_text("{}", encoding="utf-8")
    recordings = tmp_path / "recordings"
    recordings.mkdir(exist_ok=True)
    profile = replace(CapabilityProfile.full(), allowed_tools=(
        CapabilityProfile.full().allowed_tools | {"list_assets", "replay_recording", "stop"}
    ))
    service = RobotBrokerService(
        executor=executor, token="fake", event_path=event_path,
        allow_simulated_sdk_posts=True, watchdog_timeout_s=2,
        replay_policy=policy, catalog_paths=(poses, recordings, sequences), profile=profile,
    )
    assert service._replay_quarantined
    service._reconcile_replay_stop("confirmed-stop", {"holding": True})
    restarted = RobotBrokerService(
        executor=executor, token="fake", event_path=event_path,
        allow_simulated_sdk_posts=True, watchdog_timeout_s=2,
        replay_policy=policy, catalog_paths=(poses, recordings, sequences), profile=profile,
    )
    assert not restarted._replay_quarantined
    assert event_path.read_bytes().count(b"\n") == 2
    request = {"request_id": "unknown-fragment", "session_id": executor.session_id,
               "asset_id": snapshot.asset_id, "revision": snapshot.revision,
               "speed_scale": 1.0}
    response = restarted._dispatch_replay("http-id", request)
    assert response.status == 409  # no lease/heartbeat in this recovered service
    assert "confirmed STOP required" not in response.body["error"]


def test_stop_between_replay_reservation_and_dispatch_cancels_reserved_epoch(
    simulation, tmp_path, monkeypatch,
):
    executor, watchdog, _auth, policy, snapshot = simulation
    event_path = tmp_path / "events"
    poses, sequences, recordings = tmp_path / "poses.json", tmp_path / "sequences.json", tmp_path / "recordings"
    poses.write_text("{}", encoding="utf-8")
    sequences.write_text("{}", encoding="utf-8")
    recordings.mkdir(exist_ok=True)
    profile = replace(CapabilityProfile.full(), allowed_tools=(
        CapabilityProfile.full().allowed_tools | {"list_assets", "replay_recording", "stop"}
    ))
    service = RobotBrokerService(executor=executor, token="fake", event_path=event_path,
        allow_simulated_sdk_posts=True, watchdog_timeout_s=2, replay_policy=policy,
        catalog_paths=(poses, recordings, sequences), profile=profile)
    original = service._execute
    entered, proceed = threading.Event(), threading.Event()
    def delayed(**kwargs):
        if kwargs["action"] == "replay_recording":
            entered.set()
            assert proceed.wait(2)
        return original(**kwargs)
    monkeypatch.setattr(service, "_execute", delayed)
    request = {"request_id": "racing-id", "session_id": executor.session_id,
        "asset_id": snapshot.asset_id, "revision": snapshot.revision, "speed_scale": 1.0}
    responses = []
    thread = threading.Thread(target=lambda: responses.append(service._dispatch_replay("http", request)))
    thread.start()
    assert entered.wait(1)
    assert service.dispatch("POST", "/v1/stop", {}).status == 200
    proceed.set()
    thread.join(2)
    assert not thread.is_alive()
    assert responses[0].status == 409
    assert "newer STOP" in responses[0].body["error"]
    assert executor._session.get_state().torque_enabled is False


def test_simulated_recording_requires_exact_pin_without_human_lease(simulation):
    executor, watchdog, auth, policy, snapshot = simulation
    auth.clear()
    watchdog.heartbeat()
    unpinned = replace(policy, allowed_assets=(("soarm101/sequence/other", "sha256:" + "0" * 64),))
    with pytest.raises(PermissionError, match="not allowed"):
        replay_simulated_recording(executor, watchdog, unpinned, snapshot, speed_scale=1.0)


@pytest.mark.parametrize("deadline", [False, True])
def test_stop_or_deadline_rejects_late_success(simulation, monkeypatch, deadline):
    executor, watchdog, auth, policy, snapshot = simulation
    auth.issue(robot_id="so101", calibration_id="simulation", minutes=1)
    watchdog.heartbeat()
    entered, release = threading.Event(), threading.Event()
    errors = []
    def fake_play(*args, **kwargs):
        entered.set()
        assert release.wait(2)
        return MotionResult(True, True)
    monkeypatch.setattr(executor._session, "play_trajectory", fake_play)
    if deadline:
        policy = replace(policy, max_duration_s=0.2)
    def run():
        try:
            replay_simulated_recording(executor, watchdog, policy, snapshot, speed_scale=1.0)
        except Exception as exc:
            errors.append(exc)
    thread = threading.Thread(target=run)
    thread.start()
    assert entered.wait(1)
    if deadline:
        # The simulated controller is deliberately held until the watchdog
        # records the deadline-triggered STOP.
        for _ in range(100):
            if watchdog.status["state"] == "stopped":
                break
            threading.Event().wait(0.01)
        assert watchdog.status["state"] == "stopped"
    else:
        watchdog.begin_stop()
        watchdog.finish_stop(result=executor.execute("stop", {}))
    release.set()
    thread.join(1)
    assert not thread.is_alive()
    assert errors and ("deadline" in str(errors[0]) or "terminal" in str(errors[0]))


def test_lost_supervisor_heartbeat_trips_and_rejects_late_completion(simulation, monkeypatch):
    executor, watchdog, auth, policy, snapshot = simulation
    auth.issue(robot_id="so101", calibration_id="simulation", minutes=1)
    watchdog.heartbeat()
    entered, release = threading.Event(), threading.Event()
    errors = []
    def fake_play(*args, **kwargs):
        entered.set()
        assert release.wait(2)
        return MotionResult(True, True)
    monkeypatch.setattr(executor._session, "play_trajectory", fake_play)
    def run():
        try:
            replay_simulated_recording(executor, watchdog, policy, snapshot, speed_scale=1.0)
        except Exception as exc:
            errors.append(exc)
    thread = threading.Thread(target=run)
    thread.start()
    assert entered.wait(1)
    with watchdog._lock:
        watchdog._deadline = watchdog._clock() - 1
    with pytest.raises(PermissionError, match="heartbeat expired"):
        watchdog.check()
    for _ in range(100):
        if watchdog.status["state"] == "tripped":
            break
        threading.Event().wait(0.01)
    assert watchdog.status["state"] == "tripped"
    release.set()
    thread.join(1)
    assert not thread.is_alive()
    assert errors


def test_stop_during_preflight_does_not_enable_simulated_torque(simulation, monkeypatch):
    executor, watchdog, auth, policy, snapshot = simulation
    auth.issue(robot_id="so101", calibration_id="simulation", minutes=1)
    watchdog.heartbeat()
    entered, release = threading.Event(), threading.Event()
    original = ReplayPolicy.check_start
    errors = []
    def delayed_check(self, actual, expected):
        entered.set()
        assert release.wait(2)
        return original(self, actual, expected)
    monkeypatch.setattr(ReplayPolicy, "check_start", delayed_check)
    def run():
        try:
            replay_simulated_recording(executor, watchdog, policy, snapshot, speed_scale=1.0)
        except Exception as exc:
            errors.append(exc)
    thread = threading.Thread(target=run)
    thread.start()
    assert entered.wait(1)
    watchdog.begin_stop()
    watchdog.finish_stop(result=executor.execute("stop", {}))
    release.set()
    thread.join(1)
    assert not thread.is_alive() and errors
    assert not executor._session.get_state().torque_enabled


@pytest.mark.parametrize("stop_fails", [False, True])
def test_playback_failure_reports_stop_outcome(simulation, monkeypatch, stop_fails):
    executor, watchdog, auth, policy, snapshot = simulation
    auth.issue(robot_id="so101", calibration_id="simulation", minutes=1)
    watchdog.heartbeat()
    def fail_play(*args, **kwargs):
        raise RuntimeError("injected playback failure")
    monkeypatch.setattr(executor._session, "play_trajectory", fail_play)
    if stop_fails:
        def fail_stop():
            raise RuntimeError("injected stop failure")
        monkeypatch.setattr(executor._session, "stop", fail_stop)
    with pytest.raises(ReplayExecutionError) as caught:
        replay_simulated_recording(executor, watchdog, policy, snapshot, speed_scale=1.0)
    assert str(caught.value.original) == "injected playback failure"
    assert caught.value.stop["outcome"] == ("unconfirmed" if stop_fails else "confirmed")
    assert auth.status()["armed"] is False
