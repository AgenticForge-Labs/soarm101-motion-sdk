"""MCP adapter contract tests; never connect to physical robot hardware."""

from __future__ import annotations

import asyncio
import base64
import hashlib
import json
import math

import pytest

from soarm101_motion.mcp_server import _finite, _verified_capture, create_server


def test_verified_capture_checks_hash_and_strips_host_metadata() -> None:
    raw = b"example-camera-jpeg"
    sha = hashlib.sha256(raw).hexdigest()
    response = {
        "ok": True,
        "request_id": "capture123",
        "result": {
            "name": "overhead",
            "width": 640,
            "height": 480,
            "sha256": sha,
            "image_base64": base64.b64encode(raw).decode(),
            "device": "/dev/video0",
            "path": "/private/host/image.jpg",
        },
    }
    metadata, encoded = _verified_capture(response)
    assert encoded == base64.b64encode(raw).decode()
    assert metadata == {
        "request_id": "capture123",
        "name": "overhead",
        "width": 640,
        "height": 480,
        "sha256": sha,
    }
    response["result"]["sha256"] = "0" * 64
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        _verified_capture(response)


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf, True])
def test_finite_rejects_unsafe_numbers(bad: float) -> None:
    with pytest.raises(ValueError, match="finite number"):
        _finite(bad, "x_mm")


def test_mcp_tools_match_bounded_broker_contract() -> None:
    pytest.importorskip("mcp")
    from mcp import Client

    calls: list[tuple[str, str, object]] = []
    raw = b"jpeg-evidence"
    sha = hashlib.sha256(raw).hexdigest()

    def request(*, method: str, path: str, payload=None):
        calls.append((method, path, payload))
        if path == "/v1/capture":
            result = {
                "name": "overhead",
                "sha256": sha,
                "image_base64": base64.b64encode(raw).decode(),
            }
        else:
            result = {"completed": True, "holding": True}
        return {"ok": True, "request_id": str(len(calls)), "result": result}

    server = create_server(request)

    async def run() -> None:
        async with Client(server) as client:
            tools = {tool.name for tool in (await client.list_tools()).tools}
            assert tools == {
                "robot_health", "robot_capabilities", "robot_state",
                "capture_camera", "go_pose", "jog_joint", "jog_cartesian",
                "move_gripper", "sleep", "sleep_up", "stop",
            }
            assert not tools.intersection({
                "arm", "disarm", "relax", "calibrate", "raw_joint", "arbitrary_shell",
            })
            state = await client.call_tool("robot_state", {})
            assert not state.is_error
            joint = await client.call_tool(
                "jog_joint", {"joint": "elbow_flex", "delta_deg": 5}
            )
            assert not joint.is_error
            jog = await client.call_tool(
                "jog_cartesian", {"frame": "tool", "x_mm": 1, "y_mm": -2, "z_mm": 3}
            )
            assert not jog.is_error
            capture = await client.call_tool("capture_camera", {"camera": "overhead"})
            assert not capture.is_error
            texts = [item.text for item in capture.content if item.type == "text"]
            images = [item.data for item in capture.content if item.type == "image"]
            assert len(images) == 1 and base64.b64decode(images[0]) == raw
            assert len(texts) == 1
            metadata = json.loads(texts[0])
            assert metadata["sha256"] == sha
            assert "image_base64" not in texts[0]
            assert "/dev/" not in texts[0]
            stop = await client.call_tool("stop", {})
            assert not stop.is_error
            invalid = await client.call_tool(
                "jog_cartesian", {"frame": "camera", "x_mm": 3}
            )
            assert invalid.is_error

    asyncio.run(run())
    assert calls == [
        ("GET", "/v1/state", None),
        ("POST", "/v1/joint", {"joint": "elbow_flex", "delta_deg": 5.0}),
        ("POST", "/v1/jog", {"frame": "tool", "x_mm": 1.0, "y_mm": -2.0, "z_mm": 3.0}),
        ("POST", "/v1/capture", {"camera": "overhead"}),
        ("POST", "/v1/stop", {}),
    ]


def test_broker_errors_are_returned_as_mcp_errors() -> None:
    pytest.importorskip("mcp")
    from mcp import Client

    def rejected(*, method: str, path: str, payload=None):
        raise RuntimeError("broker HTTP 409: motion authority expired")

    server = create_server(rejected)

    async def run() -> None:
        async with Client(server) as client:
            result = await client.call_tool("go_pose", {"name": "agent_start_overhead"})
            assert result.is_error
            assert "authority expired" in str(result.content)

    asyncio.run(run())
