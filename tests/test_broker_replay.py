"""Broker-scoped bounded replay: simulated SDK and fakes, never hardware."""
import asyncio
import threading
from dataclasses import replace

import numpy as np
import pytest

from soarm101_motion.agent_control import AgentAuthorityStore
from soarm101_motion.asset_catalog import _file_digest, _revision
from soarm101_motion.broker import (
    AgentMotionRates, BrokerRelayService, RobotBrokerHTTPServer, RobotBrokerService,
)
from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.capability_profile import CapabilityProfile
from soarm101_motion.config import SOARM101Config
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.mcp_server import create_server
from soarm101_motion.replay_policy import ReplayPolicy
from soarm101_motion.trajectories import Trajectory, TrajectoryLibrary
from soarm101_motion.types import MotionResult
from soarm101_motion.sequences import MotionSequence, SequenceLibrary, SequenceStep
from soarm101_motion.asset_catalog import discover_assets
from soarm101_motion.poses import PoseLibrary, SavedPose


def profile(*tools):
    return CapabilityProfile.from_document({"schema_version": 1, "name": "replay-test",
        "tools": list(tools), "cameras": [], "limits": {}})


@pytest.fixture
def replay(tmp_path):
    auth = AgentAuthorityStore(tmp_path / "lease.json")
    executor = SDKAgentExecutor(config=SOARM101Config(robot_id="so101"), rates=AgentMotionRates(),
        simulation=True, simulation_actions=True, authority_store=auth)
    executor.execute("state", {})
    arm = executor._session
    samples = [arm.get_joint_positions().positions[name] for name in ARM_JOINTS]
    gripper = arm.tool.get_position()
    library = TrajectoryLibrary("so101", root=tmp_path / "recordings")
    entry = library.save("still", Trajectory(timestamps_s=np.array([0., 0.1]),
        joints_rad=np.array([samples, samples]), gripper=np.array([gripper, gripper]),
        metadata={"source_robot_id": "so101", "source_calibration_id": "simulation"}), kind="raw")
    revision = _revision({"data_sha256": _file_digest(entry.data_path),
                          "metadata_sha256": _file_digest(entry.metadata_path)})
    ident = "soarm101/recording/raw/still"
    policy = ReplayPolicy("so101", "simulation", ((ident, revision),), 1., 10, 5., .05, .05)
    pose, sequence = tmp_path / "poses.json", tmp_path / "sequences.json"
    pose.write_text("{}")
    sequence.write_text("{}")
    service = RobotBrokerService(executor=executor, token="fake", event_path=tmp_path / "events",
        profile=profile("robot_state", "robot_capabilities", "list_assets", "replay_recording", "stop"),
        catalog_paths=(pose, library.root, sequence), replay_policy=policy,
        allow_simulated_sdk_posts=True, watchdog_timeout_s=2)
    service.watchdog.start()
    request = {"request_id": "take-1", "session_id": executor.session_id, "asset_id": ident,
               "revision": revision, "speed_scale": 1.}
    try:
        yield service, auth, request, entry
    finally:
        service.watchdog.close()
        executor.close()


def arm_simulation(service, auth):
    auth.issue(robot_id="so101", calibration_id="simulation", minutes=1)
    service.watchdog.heartbeat()


def submit(service, request):
    return service.dispatch("POST", "/v1/replay-recording", request)


def test_pinned_sequence_executes_mixed_steps_in_order_simulation(replay, monkeypatch, tmp_path):
    service, auth, recording_request, _ = replay
    executor = service.executor
    arm = executor._session
    sequence_path = service._catalog_paths[2]
    library = SequenceLibrary("so101", path=sequence_path)
    pose_library = PoseLibrary("so101", path=service._catalog_paths[0])
    pose_library.save("checkpoint", replace(
        SavedPose.capture(arm), source_calibration_id="simulation"))
    position = float(arm.tool.get_position())
    library.save(MotionSequence("ordered", (
        SequenceStep("point", {"name": "checkpoint"}),
        SequenceStep("gripper", {"position": position, "speed_scale": 0.5}),
        SequenceStep("wait", {"seconds": 0.01}),
        SequenceStep("trajectory", {"name": "still", "kind": "raw"}),
        SequenceStep("gripper", {"position": position}),
    ), metadata={"source_robot_id": "so101", "source_calibration_id": "simulation"}))
    assets = discover_assets(
        pose_library,
        TrajectoryLibrary("so101", root=service._catalog_paths[1]), library,
        current_robot_id="so101", current_calibration_id="simulation",
    )
    descriptor = next(item for item in assets if item.id == "soarm101/sequence/ordered")
    recording = recording_request["asset_id"]
    revision = recording_request["revision"]
    service._replay_policy = ReplayPolicy(
        "so101", "simulation", ((recording, revision), (descriptor.id, descriptor.revision),
                                  ("soarm101/pose/checkpoint", next(
                                      item.revision for item in assets if item.id == "soarm101/pose/checkpoint"))),
        1., 10, 5., .05, .05,
    )
    service.profile = profile("robot_state", "robot_capabilities", "list_assets",
                              "replay_sequence", "stop")
    service.watchdog.heartbeat()
    order = []
    gripper_rates = []
    original_move = arm.tool.move
    original_play = arm.play_trajectory
    original_pose = arm.move_joints_from_saved_pose
    def move(*args, **kwargs):
        order.append("gripper")
        gripper_rates.append((kwargs["speed_raw"], kwargs["acceleration_raw"]))
        return original_move(*args, **kwargs)
    def play(*args, **kwargs):
        order.append("recording")
        return original_play(*args, **kwargs)
    def pose(*args, **kwargs):
        order.append("pose")
        return original_pose(*args, **kwargs)
    monkeypatch.setattr(arm.tool, "move", move)
    monkeypatch.setattr(arm, "play_trajectory", play)
    monkeypatch.setattr(arm, "move_joints_from_saved_pose", pose)
    request = {"request_id": "sequence-1", "session_id": executor.session_id,
               "asset_id": descriptor.id, "revision": descriptor.revision, "speed_scale": 1.}
    result = service.dispatch("POST", "/v1/replay-sequence", request)
    assert result.status == 200, result.body
    assert result.body["result"]["completed"] is True
    assert [s["kind"] for s in result.body["result"]["steps"]] == [
        "point", "gripper", "wait", "trajectory", "gripper",
    ]
    assert order == ["pose", "gripper", "recording", "gripper"]
    assert gripper_rates == [(125, 10), (250, 20)]
    assert service.dispatch("POST", "/v1/replay-sequence", request) == result


@pytest.mark.parametrize("interrupt_kind", ["stop", "heartbeat-expiry"])
def test_sequence_stop_or_lease_loss_during_pose_fences_late_success(
    replay, monkeypatch, interrupt_kind,
):
    service, auth, _, _ = replay
    executor = service.executor
    arm = executor._session
    poses = PoseLibrary("so101", path=service._catalog_paths[0])
    poses.save("checkpoint", replace(
        SavedPose.capture(arm), source_calibration_id="simulation"))
    sequences = SequenceLibrary("so101", path=service._catalog_paths[2])
    sequences.save(MotionSequence("stop-race", (
        SequenceStep("point", {"name": "checkpoint"}),
    ), metadata={"source_robot_id": "so101", "source_calibration_id": "simulation"}))
    trajectories = TrajectoryLibrary("so101", root=service._catalog_paths[1])
    assets = discover_assets(poses, trajectories, sequences,
                             current_robot_id="so101", current_calibration_id="simulation")
    descriptor = next(item for item in assets if item.id == "soarm101/sequence/stop-race")
    pose_descriptor = next(item for item in assets if item.id == "soarm101/pose/checkpoint")
    service._replay_policy = ReplayPolicy(
        "so101", "simulation", ((descriptor.id, descriptor.revision),
                                  (pose_descriptor.id, pose_descriptor.revision)),
        1., 10, 5., .05, .05,
    )
    service.profile = profile("robot_state", "list_assets", "replay_sequence", "stop")
    arm_simulation(service, auth)
    original_pose = arm.move_joints_from_saved_pose
    def interrupt_pose(*args, **kwargs):
        result = original_pose(*args, **kwargs)
        if interrupt_kind == "stop":
            stopped = service.dispatch("POST", "/v1/stop", {})
            assert stopped.status == 200
        else:
            with service.watchdog._lock:
                service.watchdog._deadline = service.watchdog._clock() - 1
            with pytest.raises(PermissionError, match="heartbeat expired"):
                service.watchdog.check()
        return result
    monkeypatch.setattr(arm, "move_joints_from_saved_pose", interrupt_pose)
    request = {"request_id": "sequence-stop-race", "session_id": executor.session_id,
               "asset_id": descriptor.id, "revision": descriptor.revision, "speed_scale": 1.}
    response = service.dispatch("POST", "/v1/replay-sequence", request)
    assert response.status == 409
    assert "terminal" in response.body["error"] or "authority" in response.body["error"]
    assert response.body.get("result", {}).get("completed") is not True


def test_stop_after_home_pose_does_not_submit_gripper_step(replay, monkeypatch):
    service, auth, _, _ = replay
    executor = service.executor
    arm = executor._session
    poses = PoseLibrary("so101", path=service._catalog_paths[0])
    poses.save("home", replace(
        SavedPose.capture(arm), source_calibration_id="simulation"))
    sequences = SequenceLibrary("so101", path=service._catalog_paths[2])
    sequences.save(MotionSequence("home-stop-race", (
        SequenceStep("home", {}),
    ), metadata={"source_robot_id": "so101", "source_calibration_id": "simulation"}))
    trajectories = TrajectoryLibrary("so101", root=service._catalog_paths[1])
    assets = discover_assets(poses, trajectories, sequences,
                             current_robot_id="so101", current_calibration_id="simulation")
    descriptor = next(item for item in assets if item.id == "soarm101/sequence/home-stop-race")
    pose_descriptor = next(item for item in assets if item.id == "soarm101/pose/home")
    service._replay_policy = ReplayPolicy(
        "so101", "simulation", ((descriptor.id, descriptor.revision),
                                  (pose_descriptor.id, pose_descriptor.revision)),
        1., 10, 5., .05, .05,
    )
    service.profile = profile("robot_state", "list_assets", "replay_sequence", "stop")
    arm_simulation(service, auth)
    original_pose = arm.move_joints_from_saved_pose

    def stop_after_pose(*args, **kwargs):
        result = original_pose(*args, **kwargs)
        stopped = service.dispatch("POST", "/v1/stop", {})
        assert stopped.status == 200
        return result

    def forbidden_gripper(*args, **kwargs):
        pytest.fail("gripper command submitted after STOP")

    monkeypatch.setattr(arm, "move_joints_from_saved_pose", stop_after_pose)
    monkeypatch.setattr(arm.tool, "move", forbidden_gripper)
    request = {"request_id": "home-stop-race", "session_id": executor.session_id,
               "asset_id": descriptor.id, "revision": descriptor.revision, "speed_scale": 1.}
    response = service.dispatch("POST", "/v1/replay-sequence", request)
    assert response.status == 409
    assert response.body.get("result", {}).get("completed") is not True


def test_grants_cannot_enable_default_or_physical_or_unsupervised_replay(replay, tmp_path):
    service, _, request, _ = replay
    assert "replay_recording" not in CapabilityProfile.full().allowed_tools
    assert submit(RobotBrokerService(token="fake", event_path=tmp_path / "denied"), request).status == 403
    with pytest.raises(ValueError, match="simulation actions only"):
        RobotBrokerService(token="fake", profile=profile("replay_recording"))
    with pytest.raises(ValueError, match="simulation actions only"):
        RobotBrokerService(token="fake", executor=service.executor,
            profile=profile("replay_recording"), replay_policy=service._replay_policy)
    relay = BrokerRelayService(backend=service, token="relay", agent_profile=profile("robot_state"))
    assert submit(relay, request).status == 403
    physical = SDKAgentExecutor(config=SOARM101Config(), rates=AgentMotionRates(),
                                arm_factory=lambda: pytest.fail("must never open hardware"))
    with pytest.raises(ValueError, match="simulation actions only"):
        RobotBrokerService(token="fake", executor=physical, profile=service.profile,
            replay_policy=service._replay_policy, catalog_paths=service._catalog_paths)
    assert physical.connection_count == 0


def test_pinned_simulation_replay_uses_profile_authority_without_physical_lease(replay):
    service, _auth, request, _ = replay
    service.watchdog.heartbeat()
    result = submit(service, request)
    assert result.status == 200, result.body
    assert result.body["result"]["mode"] == "simulation"
    assert service.executor._authority.status()["armed"] is False


def test_replay_reuses_session_forwards_rates_and_deduplicates(replay, monkeypatch):
    service, auth, request, _ = replay
    service.watchdog.heartbeat()
    executor = service.executor
    arm = executor._session
    writes = []
    original = arm.backend.write_tool_position
    def capture(*args, **kwargs):
        writes.append(kwargs)
        return original(*args, **kwargs)
    monkeypatch.setattr(arm.backend, "write_tool_position", capture)
    result = submit(service, request)
    print(result.body.get("error"))
    assert result.status == 200, result.body
    assert result.body["result"]["revision"] == request["revision"]
    assert result.body["result"]["broker_session"]["id"] == executor.session_id
    assert executor.connection_count == 1 and executor._session is arm
    count = len(writes)
    assert count and all(w["speed_raw"] == 250 and w["acceleration_raw"] == 20 for w in writes)
    assert submit(service, request) == result
    assert len(writes) == count
    assert submit(service, {**request, "speed_scale": .5}).status == 409
    assert submit(service, {**request, "session_id": "prior-session"}).status == 409


@pytest.mark.parametrize("changes", [
    {"speed_scale": True}, {"speed_scale": 1.1}, {"speed_scale": float("nan")},
    {"path": "/tmp/asset"}, {"gripper_speed_raw": 3400}, {"request_id": "../x"},
    {"asset_id": "soarm101/sequence/anything"},
])
def test_strict_recording_payload(replay, changes):
    service, _, request, _ = replay
    assert submit(service, {**request, **changes}).status == 400
    assert not service.executor._session.get_state().torque_enabled


def test_pinned_revision_and_mutation_rejected_before_enable(replay):
    service, auth, request, entry = replay
    arm_simulation(service, auth)
    assert submit(service, {**request, "revision": "sha256:" + "0" * 64}).status == 403
    entry.metadata_path.write_bytes(entry.metadata_path.read_bytes() + b" ")
    assert submit(service, request).status == 409
    assert not service.executor._session.get_state().torque_enabled


def repin(service, request, entry):
    revision = _revision({"data_sha256": _file_digest(entry.data_path),
                          "metadata_sha256": _file_digest(entry.metadata_path)})
    service._replay_policy = replace(service._replay_policy,
                                     allowed_assets=((request["asset_id"], revision),))
    return {**request, "revision": revision}


def test_calibration_mismatch_is_denied_even_with_new_pinned_hash(replay):
    import json
    service, auth, request, entry = replay
    arm_simulation(service, auth)
    metadata = json.loads(entry.metadata_path.read_text())
    metadata["metadata"]["source_calibration_id"] = "another-calibration"
    entry.metadata_path.write_text(json.dumps(metadata))
    response = submit(service, repin(service, request, entry))
    assert response.status == 409
    assert not service.executor._session.get_state().torque_enabled


def test_broker_joint_rates_apply_to_recorded_frames(replay):
    service, auth, request, entry = replay
    arm_simulation(service, auth)
    library = TrajectoryLibrary("so101", root=service._catalog_paths[1])
    trajectory = library.load("still", kind="raw")
    samples = trajectory.joints_rad.copy()
    samples[-1, 0] += .001
    entry.data_path.unlink()
    entry.metadata_path.unlink()
    library.save("still", replace(trajectory, joints_rad=samples), kind="raw")
    service.executor.rates = replace(service.executor.rates, joint_speed_deg_s=.01)
    response = submit(service, repin(service, request, entry))
    assert response.status == 409
    assert response.body["stop"]["outcome"] == "confirmed"
    assert not auth.status()["armed"]


def test_request_capacity_has_no_eviction(replay):
    service, _, request, _ = replay
    service._replay_requests.update({str(i): ({}, None) for i in range(1000)})
    response = submit(service, request)
    assert response.status == 409 and "capacity" in response.body["error"]
    assert len(service._replay_requests) == 1000


def test_stop_unconfirmed_is_exposed_and_retained(replay, monkeypatch):
    service, auth, request, _ = replay
    arm_simulation(service, auth)
    def fail(*args, **kwargs):
        raise RuntimeError("injected failure")
    monkeypatch.setattr(service.executor._session, "play_trajectory", fail)
    monkeypatch.setattr(service.executor._session, "stop", fail)
    response = submit(service, request)
    assert response.status == 409
    assert response.body["stop"]["outcome"] == "unconfirmed"
    assert submit(service, request) == response
    assert not auth.status()["armed"]


def test_stop_invalidates_queued_replay(replay):
    service, auth, request, _ = replay
    arm_simulation(service, auth)
    response = []
    service._operation_lock.acquire()
    thread = threading.Thread(target=lambda: response.append(submit(service, request)))
    try:
        thread.start()
        # Reservation happens before waiting for the ordinary operation lock.
        for _ in range(100):
            with service._replay_lock:
                pending = bool(service._replay_requests)
            if pending:
                break
            threading.Event().wait(.01)
        assert pending
        assert submit(service, request).status == 409
        assert service.dispatch("POST", "/v1/stop", {}).status == 200
    finally:
        service._operation_lock.release()
        thread.join(2)
    assert not thread.is_alive() and response[0].status == 409
    assert not auth.status()["armed"]
    assert not service.executor._session.get_state().torque_enabled


@pytest.mark.parametrize("cause", ["stop", "heartbeat", "deadline", "failure"])
def test_replay_rejects_late_success_and_reports_cleanup(replay, monkeypatch, cause):
    service, auth, request, _ = replay
    arm_simulation(service, auth)
    entered, release = threading.Event(), threading.Event()
    responses = []
    def fake_play(*args, **kwargs):
        entered.set()
        assert release.wait(2)
        if cause == "failure":
            raise RuntimeError("injected playback failure")
        return MotionResult(True, True)
    monkeypatch.setattr(service.executor._session, "play_trajectory", fake_play)
    if cause == "deadline":
        service._replay_policy = replace(service._replay_policy, max_duration_s=.2)
    thread = threading.Thread(target=lambda: responses.append(submit(service, request)))
    thread.start()
    try:
        assert entered.wait(1)
        if cause == "stop":
            assert service.dispatch("POST", "/v1/stop", {}).status == 200
        elif cause == "heartbeat":
            with service.watchdog._lock:
                service.watchdog._deadline = service.watchdog._clock() - 1
            with pytest.raises(PermissionError, match="heartbeat expired"):
                service.watchdog.check()
        if cause in {"heartbeat", "deadline"}:
            for _ in range(100):
                if service.watchdog.status["state"] in {"tripped", "stopped"}:
                    break
                threading.Event().wait(.01)
            assert service.watchdog.status["state"] in {"tripped", "stopped"}
    finally:
        release.set()
        thread.join(2)
    assert not thread.is_alive() and responses[0].status == 409
    assert responses[0].body["stop"]["outcome"] in {"confirmed", "unconfirmed"}
    assert not auth.status()["armed"]
    assert submit(service, {**request, "request_id": "take-2"}).status == 409


def test_mcp_replay_tool_is_explicit_and_forwards_identity():
    pytest.importorskip("mcp")
    from mcp import Client
    calls = []
    def call(**kwargs):
        calls.append(kwargs)
        return {"ok": True, "result": {}}
    async def run():
        async with Client(create_server(call)) as client:
            assert "replay_recording" not in {t.name for t in (await client.list_tools()).tools}
        async with Client(create_server(call, allowed_tools={"replay_recording"})) as client:
            result = await client.call_tool("replay_recording", {"request_id": "a", "session_id": "b",
                "asset_id": "soarm101/recording/raw/still", "revision": "r", "speed_scale": .5})
            assert not result.is_error
    asyncio.run(run())
    assert calls == [{"method": "POST", "path": "/v1/replay-recording", "payload": {
        "request_id": "a", "session_id": "b", "asset_id": "soarm101/recording/raw/still",
        "revision": "r", "speed_scale": .5}}]


def test_robotctl_replay_over_authenticated_simulated_http(replay, monkeypatch, capsys):
    from soarm101_motion import robotctl
    import json
    service, auth, request, _ = replay
    arm_simulation(service, auth)
    server = RobotBrokerHTTPServer(("127.0.0.1", 0), service)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    monkeypatch.setenv("SOARM101_BROKER_URL", f"http://127.0.0.1:{server.server_port}")
    monkeypatch.setenv("SOARM101_BROKER_TOKEN", "fake")
    thread.start()
    args = ["replay-recording", request["asset_id"], "--request-id", request["request_id"],
            "--session-id", request["session_id"], "--revision", request["revision"],
            "--speed-scale", "1"]
    try:
        assert robotctl.main(args) == 0
        first = json.loads(capsys.readouterr().out)
        assert first["result"]["completion"]["completed"]
        assert robotctl.main(args) == 0
        assert json.loads(capsys.readouterr().out) == first
    finally:
        server.shutdown()
        server.server_close()
        thread.join(2)
