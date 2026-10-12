"""Broker supervisor watchdog regressions; simulation and fake time only."""

from __future__ import annotations

import json
import threading
import urllib.error
import urllib.request

import pytest

from soarm101_motion.agent_control import AgentAuthorityStore
from soarm101_motion.broker import (
    AgentMotionRates,
    BrokerRelayHTTPServer,
    BrokerRelayService,
    RobotBrokerHTTPServer,
    RobotBrokerService,
)
from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.capability_profile import CapabilityProfile
from soarm101_motion.config import SOARM101Config
from soarm101_motion.supervisor_watchdog import SupervisorHeartbeatWatchdog


def test_watchdog_is_latched_and_stops_once_after_lost_host() -> None:
    now = [100.0]
    calls = []
    watchdog = SupervisorHeartbeatWatchdog(
        timeout_s=3,
        on_expire=lambda: calls.append("stop"),
        clock=lambda: now[0],
    )
    with pytest.raises(PermissionError, match="heartbeat missing"):
        watchdog.require_fresh()
    assert watchdog.pulse()["state"] == "armed"
    watchdog.require_fresh()
    now[0] += 2.5
    watchdog.pulse()
    now[0] += 2
    assert watchdog.check() is False
    now[0] += 1.1
    with pytest.raises(PermissionError, match="stale"):
        watchdog.require_fresh()
    assert watchdog.check() is True
    assert calls == ["stop"]
    assert watchdog.check() is False
    with pytest.raises(PermissionError, match="latched"):
        watchdog.pulse()
    assert watchdog.snapshot()["state"] == "expired"


def test_manual_stop_disarms_heartbeat_until_fresh_broker() -> None:
    watchdog = SupervisorHeartbeatWatchdog(
        timeout_s=3, on_expire=lambda: pytest.fail("unexpected watchdog STOP"),
    )
    watchdog.pulse()
    watchdog.stop()
    assert watchdog.check() is False
    with pytest.raises(PermissionError, match="latched"):
        watchdog.pulse()
    with pytest.raises(PermissionError, match="heartbeat"):
        watchdog.require_fresh()


def test_watchdog_lost_host_triggers_private_stop_and_denies_agent_heartbeat(
    tmp_path,
) -> None:
    now = [50.0]
    lease = AgentAuthorityStore(tmp_path / "authority.json")
    lease.issue(robot_id="so101", calibration_id="simulation", minutes=5)
    executor = SDKAgentExecutor(
        config=SOARM101Config(robot_id="so101"),
        rates=AgentMotionRates(),
        simulation=True, simulation_actions=True, authority_store=lease,
    )
    agent_profile = CapabilityProfile.from_document({
        "schema_version": 1, "name": "no-host-control",
        "tools": ["robot_state", "robot_capabilities", "jog_joint"],
        "cameras": [], "limits": {"max_joint_delta_deg": 1},
    })
    doc = agent_profile.as_dict()
    doc["tools"] = sorted(agent_profile.allowed_tools | {"stop"})
    service = RobotBrokerService(
        executor=executor, token="trusted-only",
        profile=CapabilityProfile.from_document(doc),
        allow_simulated_sdk_posts=True, event_path=tmp_path / "events.jsonl",
    )
    outcomes = []

    def expire_stop():
        outcome = service.dispatch("POST", "/v1/stop", {})
        outcomes.append(outcome)
        if outcome.status != 200:
            raise RuntimeError("watchdog stop failed")

    watchdog = SupervisorHeartbeatWatchdog(
        timeout_s=3, on_expire=expire_stop, clock=lambda: now[0],
    )
    service.attach_supervisor_watchdog(watchdog)
    private = RobotBrokerHTTPServer(("127.0.0.1", 0), service)
    external = BrokerRelayHTTPServer(
        ("127.0.0.1", 0),
        BrokerRelayService(
            backend=service, agent_profile=agent_profile, token="agent-only",
        ),
    )
    threads = [
        threading.Thread(target=server.serve_forever, daemon=True)
        for server in (private, external)
    ]
    for t in threads:
        t.start()

    def post(server, path, token, payload=None):
        request = urllib.request.Request(
            "http://127.0.0.1:" + str(server.server_address[1]) + path,
            method="POST",
            headers={
                "Authorization": "Bearer " + token,
                "Content-Type": "application/json",
            },
            data=json.dumps(payload or {}).encode(),
        )
        try:
            with urllib.request.urlopen(request, timeout=4) as reply:
                return reply.status, json.loads(reply.read())
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read())

    try:
        assert service.dispatch("GET", "/v1/state").status == 200
        move = {"joint": "shoulder_pan", "delta_deg": 0.2}
        assert post(external, "/v1/joint", "agent-only", move)[0] == 409
        assert post(
            external, "/v1/supervisor-heartbeat", "agent-only",
        )[0] == 403
        assert post(
            private, "/v1/supervisor-heartbeat", "agent-only",
        )[0] == 401
        assert post(
            private, "/v1/supervisor-heartbeat", "trusted-only",
        )[0] == 200
        assert post(external, "/v1/joint", "agent-only", move)[0] == 200
        now[0] += 3.1
        assert watchdog.check() is True
        assert len(outcomes) == 1
        assert outcomes[0].status == 200
        assert not lease.status()["armed"]
        assert post(external, "/v1/joint", "agent-only", move)[0] == 409
        assert post(
            private, "/v1/supervisor-heartbeat", "trusted-only",
        )[0] == 409
        assert watchdog.snapshot()["state"] == "expired"
        events = [
            json.loads(s) for s in (tmp_path / "events.jsonl").read_text().splitlines()
        ]
        assert any(r["action"] == "stop" and r["ok"] for r in events)
    finally:
        for server in (external, private):
            server.shutdown()
            server.server_close()
        for t in threads:
            t.join(timeout=3)
        executor.close()


def test_watchdog_failed_stop_keeps_fault_latched() -> None:
    now = [0.0]

    def failed_stop() -> None:
        raise RuntimeError("bus unavailable")

    watchdog = SupervisorHeartbeatWatchdog(
        timeout_s=1, clock=lambda: now[0], on_expire=failed_stop,
    )
    watchdog.pulse()
    now[0] = 1.5
    assert watchdog.check() is True
    assert "bus unavailable" in watchdog.snapshot()["last_error"]
    with pytest.raises(PermissionError):
        watchdog.require_fresh()
    assert watchdog.check() is False
