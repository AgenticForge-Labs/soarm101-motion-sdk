"""Profile validation, broker enforcement, and MCP discovery are hardware-free."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest

from soarm101_motion.broker import RobotBrokerService
from soarm101_motion.capability_profile import CapabilityProfile, ProfileError
from soarm101_motion.mcp_server import broker_visible_tools, create_server


def document(**overrides):
    body = {
        "schema_version": 1,
        "name": "coordinate-small",
        "tools": ["robot_health", "robot_capabilities", "robot_state",
                  "capture_camera", "jog_cartesian", "stop"],
        "cameras": ["overhead"],
        "limits": {"max_model_jog_mm": 5.0},
    }
    body.update(overrides)
    return body


def test_profile_round_trip_digest_and_immutable_permissions(tmp_path: Path) -> None:
    data = document()
    path = tmp_path / "profile.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    profile = CapabilityProfile.from_file(path)
    assert profile.name == "coordinate-small"
    assert profile.allowed_cameras == frozenset({"overhead"})
    assert profile.max_model_jog_mm == 5.0
    assert profile.public()["sha256"] == CapabilityProfile.from_document(data).public()["sha256"]
    with pytest.raises((AttributeError, TypeError)):
        profile.allowed_tools.add("arm")  # type: ignore[attr-defined]


@pytest.mark.parametrize("patch", [
    {"schema_version": 2},
    {"schema_version": True},
    {"name": "../danger"},
    {"tools": ["jog_cartesian", "arm"]},
    {"tools": ["robot_state", "robot_state"]},
    {"tools": "jog_cartesian"},
    {"cameras": ["secret-camera"]},
    {"cameras": ["wrist", "wrist"]},
    {"tools": ["robot_state"], "cameras": ["overhead"]},
    {"tools": ["capture_camera"], "cameras": []},
    {"limits": {"max_model_jog_mm": -1}},
    {"limits": {"max_model_jog_mm": 51}},
    {"limits": {"max_model_jog_mm": True}},
    {"limits": {"max_joint_delta_deg": 3}},
    {"limits": {"unknown_limit": 3}},
    {"unrecognized_field": 1},
])
def test_profile_rejects_invalid_or_authority_broadening_data(patch) -> None:
    with pytest.raises(ProfileError):
        CapabilityProfile.from_document(document(**patch))


class FakeExecutor:
    robot_id = "so101"

    def __init__(self):
        self.calls: list[tuple[str, ...]] = []

    def run(self, arguments):
        self.calls.append(tuple(arguments))
        if arguments[0] == "capabilities":
            return {
                "cameras": ["overhead", "wrist"],
                "actions": {
                    "state": "read_only",
                    "capture": ["overhead", "wrist"],
                    "jog": {"frames": ["world", "tool"]},
                    "joint": {"max_abs_delta_deg": 30},
                    "stop": "always_available",
                },
            }
        return {"completed": True, "holding": True}


def test_broker_denies_hidden_routes_cameras_and_oversized_jogs(tmp_path: Path) -> None:
    executor = FakeExecutor()
    profile = CapabilityProfile.from_document(document())
    service = RobotBrokerService(
        executor=executor,  # type: ignore[arg-type]
        token="secret",
        event_path=tmp_path / "events.jsonl",
        profile=profile,
    )
    exposed = service.dispatch("GET", "/v1/profile")
    assert exposed.status == 200
    assert exposed.body["result"] == profile.public()
    described = service.dispatch("GET", "/v1/capabilities")
    assert described.status == 200
    actual = described.body["result"]
    assert actual["cameras"] == ["overhead"]
    assert actual["actions"]["capture"] == ["overhead"]
    assert "joint" not in actual["actions"]

    baseline = len(executor.calls)
    assert service.dispatch("POST", "/v1/joint", {
        "joint": "shoulder_pan", "delta_deg": 2,
    }).status == 403
    assert service.dispatch("POST", "/v1/capture", {"camera": "wrist"}).status == 403
    assert service.dispatch("POST", "/v1/jog", {
        "frame": "world", "x_mm": 6.0,
    }).status == 403
    assert service.dispatch("POST", "/v1/jog", {
        "frame": "world", "x_mm": float("nan"),
    }).status == 400
    assert len(executor.calls) == baseline
    assert service.dispatch("POST", "/v1/jog", {
        "frame": "world", "x_mm": 3.0, "y_mm": 4.0,
    }).status == 200
    assert service.dispatch("POST", "/v1/stop", {}).status == 200
    audit = (tmp_path / "events.jsonl").read_text(encoding="utf-8")
    assert "profile_rejection" in audit


def test_read_only_profile_enforced_by_broker_not_just_mcp(tmp_path: Path) -> None:
    profile = CapabilityProfile.from_document({
        "schema_version": 1, "name": "read-only",
        "tools": ["robot_health", "robot_state", "capture_camera"],
        "cameras": ["overhead"], "limits": {},
    })
    executor = FakeExecutor()
    service = RobotBrokerService(
        executor=executor,  # type: ignore[arg-type]
        token="secret", event_path=tmp_path / "events.jsonl", profile=profile,
    )
    assert service.dispatch("GET", "/v1/state").status == 200
    assert service.dispatch("POST", "/v1/stop", {}).status == 403
    assert service.dispatch("POST", "/v1/gripper", {"target": "open"}).status == 403
    assert service.dispatch("POST", "/v1/sleep_up", {}).status == 403
    assert executor.calls == [("state", "--robot-id", "so101")]


def test_mcp_lists_only_broker_authorized_tools() -> None:
    pytest.importorskip("mcp")
    from mcp import Client

    profile = CapabilityProfile.from_document(document())

    def request(*, method: str, path: str, payload=None):
        assert method == "GET" and path == "/v1/profile"
        return {"ok": True, "result": profile.public()}

    enabled = broker_visible_tools(request)
    assert enabled == profile.allowed_tools
    server = create_server(request_fn=request, allowed_tools=enabled)

    async def run() -> None:
        async with Client(server) as client:
            available = {tool.name for tool in (await client.list_tools()).tools}
            assert available == enabled
            assert "jog_joint" not in available
            assert "move_gripper" not in available

    asyncio.run(run())


def test_invalid_broker_tool_list_fails_closed() -> None:
    def dishonest(*, method: str, path: str, payload=None):
        return {"ok": True, "result": {"tools": ["arm"]}}

    with pytest.raises(RuntimeError, match="unsupported"):
        broker_visible_tools(dishonest)
