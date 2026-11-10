"""Trusted-host bounded live joint stream: simulation only, no hardware."""

from __future__ import annotations

import json
import math
import random
import time

import pytest

from soarm101_motion.broker import (
    AgentMotionRates,
    RobotBrokerService,
    load_trusted_choreography_catalog,
)
from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.choreography import JointChoreography, JointFrame
from soarm101_motion.config import SOARM101Config
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.live_stream import JointEnvelope, slew_step

DEG = math.radians(1.0)
OFFSETS = tuple(6 * DEG if name in {"shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"} else 0.0
                for name in ARM_JOINTS)


def wait_for(predicate, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


@pytest.fixture
def host(tmp_path):
    executor = SDKAgentExecutor(
        config=SOARM101Config(robot_id="so101"),
        rates=AgentMotionRates(joint_speed_deg_s=50, joint_acceleration_deg_s2=500),
        simulation=True, simulation_actions=True,
    )
    executor.execute("state", {})
    origin = tuple(executor._session.get_joint_positions().positions[n] for n in ARM_JOINTS)
    envelope = JointEnvelope.author("presence", "so101", "simulation", origin, OFFSETS)
    nod = list(origin)
    nod[ARM_JOINTS.index("wrist_flex")] += 3 * DEG
    program = JointChoreography.author(
        "nod", "so101", "simulation", (JointFrame(0, origin), JointFrame(1, tuple(nod)),
                                       JointFrame(2, origin)))
    doc = {
        "policy": {"robot_id": "so101", "calibration_id": "simulation",
                   "allowed_revisions": [envelope.revision, program.revision],
                   "max_speed_deg_s": 50, "max_acceleration_deg_s2": 500, "max_duration_s": 6},
        "programs": [{"name": program.name, "robot_id": "so101", "calibration_id": "simulation",
                      "revision": program.revision,
                      "frames": [{"at_s": f.at_s, "joints_rad": list(f.joints_rad)}
                                 for f in program.frames]}],
        "envelopes": [{"name": envelope.name, "robot_id": "so101", "calibration_id": "simulation",
                       "revision": envelope.revision, "center_rad": list(origin),
                       "max_offset_rad": list(OFFSETS)}],
    }
    path = tmp_path / "catalog.json"
    path.write_text(json.dumps(doc))
    policy, catalog = load_trusted_choreography_catalog(path)
    service = RobotBrokerService(
        executor=executor, token="agent", trusted_host_token="host",
        allow_simulated_sdk_posts=True, watchdog_timeout_s=8, choreography_policy=policy,
        choreography_catalog=catalog, event_path=tmp_path / "events.jsonl")
    service.watchdog.start()
    service.watchdog.heartbeat()
    try:
        yield service, executor, envelope, program, origin
    finally:
        stream = service._live_stream
        if stream is not None and stream.active:
            try:
                stream.close()
            except Exception:
                pass
        service.watchdog.close()
        executor.close()


def post(service, route, **payload):
    return service.dispatch_trusted_host("POST", f"/v1/trusted/stream/{route}", payload)


def open_stream(service, envelope, request_id="open-1"):
    return post(service, "open", request_id=request_id, name=envelope.name, revision=envelope.revision)


def positions(executor):
    got = executor._session.get_joint_positions().positions
    return [got[name] for name in ARM_JOINTS]


def test_the_slew_limiter_never_exceeds_the_speed_or_acceleration_limits():
    rng = random.Random(7)
    dt, vmax, amax = 0.05, math.radians(50), math.radians(500)
    pos, vel, target = 0.0, 0.0, 0.0
    envelope = 6 * DEG
    prev_vel = 0.0
    for i in range(4000):
        if i % 3 == 0:
            target = rng.choice([-envelope, envelope, rng.uniform(-envelope, envelope), 0.0])
        new_pos, new_vel = slew_step(pos, vel, target, vmax, amax, dt)
        speed = (new_pos - pos) / dt
        assert abs(speed) <= vmax * 1.001
        assert abs(speed - prev_vel) <= amax * dt * 1.001
        assert abs(new_pos) <= envelope * 1.0001, "overshot the requested range"
        prev_vel, pos, vel = speed, new_pos, new_vel


def test_the_limiter_comes_to_rest_on_the_target():
    pos, vel = 0.0, 0.0
    for _ in range(200):
        pos, vel = slew_step(pos, vel, 5 * DEG, math.radians(50), math.radians(500), 0.05)
    assert pos == pytest.approx(5 * DEG, abs=1e-4) and abs(vel) < 1e-3


def test_an_envelope_is_pinned_and_bounded():
    origin = (0.0,) * 5
    envelope = JointEnvelope.author("presence", "so101", "simulation", origin, (DEG,) * 5)
    with pytest.raises(ValueError, match="content differs"):
        JointEnvelope("presence", "so101", "simulation", origin, (2 * DEG,) * 5, envelope.revision)
    with pytest.raises(ValueError, match="0..15 degrees"):
        JointEnvelope.author("wide", "so101", "simulation", origin, (16 * DEG,) * 5)
    with pytest.raises(ValueError, match="nonzero"):
        JointEnvelope.author("none", "so101", "simulation", origin, (0.0,) * 5)


def test_open_follow_clamp_and_close_return_to_the_centre(host):
    service, executor, envelope, _program, origin = host
    assert open_stream(service, envelope).status == 200
    flex = ARM_JOINTS.index("wrist_flex")
    lift = ARM_JOINTS.index("shoulder_lift")
    pan = ARM_JOINTS.index("shoulder_pan")
    try:
        reply = post(service, "offsets", offsets_rad=[0.0, 0.0, 0.0, 4 * DEG, 0.0])
        assert reply.status == 200 and reply.body["result"]["clamped"] is False
        assert wait_for(lambda: abs(positions(executor)[flex] - origin[flex] - 4 * DEG) < 0.3 * DEG)
        # Far outside the range: clamped to the pinned 6 deg, and a locked joint stays locked.
        reply = post(service, "offsets", offsets_rad=[1.0, 1.0, 0.0, 1.0, 0.0])
        assert reply.body["result"]["clamped"] is True
        assert wait_for(lambda: positions(executor)[flex] - origin[flex] > 5.5 * DEG)
        time.sleep(0.4)
        now = positions(executor)
        assert now[flex] - origin[flex] <= 6 * DEG + 1e-6
        assert now[lift] - origin[lift] <= 6 * DEG + 1e-6
        assert now[pan] == pytest.approx(origin[pan], abs=1e-9)
    finally:
        closed = post(service, "close", request_id="close-1")
    assert closed.status == 200
    assert max(abs(a - b) for a, b in zip(positions(executor), origin)) < 0.2 * DEG
    assert not executor._session.motion.is_streaming
    assert service.dispatch_trusted_host("GET", "/v1/trusted/stream", {}).body["result"]["active"] is False


def test_it_heads_back_to_the_centre_when_setpoints_stop(host):
    service, executor, envelope, _program, origin = host
    assert open_stream(service, envelope).status == 200
    flex = ARM_JOINTS.index("wrist_flex")
    post(service, "offsets", offsets_rad=[0.0, 0.0, 0.0, 5 * DEG, 0.0])
    assert wait_for(lambda: positions(executor)[flex] - origin[flex] > 4 * DEG)
    # No more setpoints: after the timeout it settles at the centre by itself.
    assert wait_for(lambda: abs(positions(executor)[flex] - origin[flex]) < 0.2 * DEG, timeout=4.0)
    assert post(service, "close", request_id="close-2").status == 200


def test_open_requires_the_exact_pinned_envelope_and_the_centre_pose(host):
    service, executor, envelope, _program, origin = host
    assert post(service, "open", request_id="a", name="presence", revision="sha256:" + "0" * 64).status == 409
    assert post(service, "open", request_id="b", name="nod", revision=envelope.revision).status == 409
    bad = post(service, "open", request_id="c", name="presence", revision=envelope.revision, extra=1)
    assert bad.status == 400
    assert service.dispatch("POST", "/v1/trusted/stream/open", {}).status == 404
    # Away from the centre pose the stream is refused rather than snapping there.
    arm = executor._session
    arm.enable()
    moved = dict(zip(ARM_JOINTS, origin))
    moved["wrist_flex"] += 5 * DEG
    arm.move_joints(moved) if hasattr(arm, "move_joints") else arm.motion.move_joints(moved)
    refused = open_stream(service, envelope, "d")
    assert refused.status == 409 and "centre" in refused.body["error"]


def test_offsets_without_a_stream_or_with_bad_values_are_refused(host):
    service, _executor, envelope, _program, _origin = host
    assert post(service, "offsets", offsets_rad=[0.0] * 5).status == 409
    assert open_stream(service, envelope).status == 200
    assert post(service, "offsets", offsets_rad=[0.0] * 4).status == 400
    assert post(service, "offsets", offsets_rad=[0.0, 0.0, 0.0, float("nan"), 0.0]).status == 400
    assert post(service, "offsets", offsets_rad=[True, 0, 0, 0, 0]).status == 400
    assert open_stream(service, envelope, "again").status == 409
    assert post(service, "close", request_id="x").status == 200


def test_a_pinned_program_cannot_run_while_the_stream_is_open(host):
    service, _executor, envelope, program, _origin = host
    assert open_stream(service, envelope).status == 200
    refused = service.dispatch_trusted_host("POST", "/v1/trusted/choreography", {
        "request_id": "nod-1", "name": program.name, "revision": program.revision})
    assert refused.status == 409
    assert post(service, "close", request_id="close-3").status == 200
    ran = service.dispatch_trusted_host("POST", "/v1/trusted/choreography", {
        "request_id": "nod-2", "name": program.name, "revision": program.revision})
    assert ran.status == 200, ran.body


def test_stop_ends_the_stream_and_further_setpoints_are_refused(host):
    service, executor, envelope, _program, _origin = host
    assert open_stream(service, envelope).status == 200
    assert service.dispatch_trusted_host("POST", "/v1/trusted/stop", {"request_id": "stop-1"}).status == 200
    stream = service._live_stream
    assert wait_for(lambda: not stream.active and stream.error is not None)
    assert post(service, "offsets", offsets_rad=[0.0] * 5).status == 409
    assert not executor._session.motion.is_streaming


def test_a_tripped_watchdog_stops_the_stream(host):
    service, executor, envelope, _program, _origin = host
    assert open_stream(service, envelope).status == 200
    service.watchdog._trip_callback("test trip")
    with service.watchdog._lock:
        service.watchdog._reason = "test trip"
    stream = service._live_stream
    assert wait_for(lambda: not stream.active and stream.error is not None)
    assert post(service, "offsets", offsets_rad=[0.0] * 5).status == 409


def test_open_and_close_are_audited(host, tmp_path):
    service, _executor, envelope, _program, _origin = host
    assert open_stream(service, envelope).status == 200
    assert post(service, "close", request_id="close-4").status == 200
    actions = [json.loads(line)["action"] for line in (tmp_path / "events.jsonl").read_text().splitlines()]
    assert "trusted_stream_open" in actions and "trusted_stream_close" in actions


def test_the_catalog_advertises_envelopes_but_not_as_targets(host):
    service, *_ = host
    listed = service.dispatch_trusted_host("GET", "/v1/trusted/catalog", {}).body["result"]
    assert [e["name"] for e in listed["envelopes"]] == ["presence"]
    assert all(t["name"] != "presence" for t in listed["targets"])
