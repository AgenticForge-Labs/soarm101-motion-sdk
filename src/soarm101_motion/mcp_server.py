"""Optional MCP tool facade for the existing authenticated SO-ARM101 agent broker.

This module does not open hardware or implement robot motion. Every call uses the
same bounded HTTP routes as robotctl. The standard CLI/GUI work without MCP installed.
"""

from __future__ import annotations

import base64
import hashlib
import hmac
import json
import math
import os
from collections.abc import Callable, Mapping
from typing import Literal

from soarm101_motion.robotctl import _request

BrokerRequest = Callable[..., dict[str, object]]
JointName = Literal[
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"
]
Frame = Literal["world", "tool"]
CameraName = Literal["overhead", "wrist"]
GripperTarget = Literal["open", "close"]


def _finite(value: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def _verified_capture(response: Mapping[str, object]) -> tuple[dict[str, object], str]:
    """Verify the broker's image SHA before returning pixels to an MCP host.

    Do not include paths, camera device names, or raw image data in text metadata.
    The broker owns capture ordering and trusted SHA evidence.
    """
    result = response.get("result")
    if not isinstance(result, dict):
        raise ValueError("broker capture response is missing result")
    encoded = result.get("image_base64")
    sha = result.get("sha256")
    if not isinstance(encoded, str) or not isinstance(sha, str) or not sha:
        raise ValueError("broker capture response is missing image or SHA-256")
    try:
        raw = base64.b64decode(encoded, validate=True)
    except (ValueError, base64.binascii.Error) as exc:
        raise ValueError("broker capture image is not valid base64") from exc
    if not hmac.compare_digest(hashlib.sha256(raw).hexdigest(), sha):
        raise ValueError("broker capture SHA-256 mismatch")
    metadata = {
        key: result[key]
        for key in ("name", "timestamp", "width", "height", "sha256")
        if key in result
    }
    metadata["request_id"] = response.get("request_id")
    return metadata, encoded


def create_server(request_fn: BrokerRequest | None = None):
    """Create an stdio MCP server bound only to the existing bounded broker API.

    Dependency injection permits tests without a robot, serial port, or running
    broker. Do not dynamically invent new routes from model-supplied arguments.
    """
    try:
        from mcp.server import MCPServer
        from mcp.types import ImageContent, TextContent, ToolAnnotations
    except ImportError as exc:
        raise RuntimeError("MCP is optional; install with pip install -e '.[mcp]'") from exc

    request = request_fn or _request
    mcp = MCPServer("SO-ARM101 bounded broker")

    def call(method: str, path: str, payload: Mapping[str, object] | None = None) -> dict[str, object]:
        response = request(method=method, path=path, payload=payload)
        if not isinstance(response, dict) or not response.get("ok"):
            raise RuntimeError("robot broker rejected the request")
        result = response.get("result")
        if not isinstance(result, dict):
            raise RuntimeError("robot broker response is missing structured result")
        return {"request_id": response.get("request_id"), "result": result}

    read_only = ToolAnnotations(read_only_hint=True, open_world_hint=False)
    action = ToolAnnotations(read_only_hint=False, open_world_hint=False)

    @mcp.tool(annotations=read_only)
    def robot_health() -> dict[str, object]:
        """Check whether the authenticated SO-ARM101 broker is responding."""
        return call("GET", "/v1/health")

    @mcp.tool(annotations=read_only)
    def robot_capabilities() -> dict[str, object]:
        """Read robot abilities, motion envelope, calibrated directions, and authority state."""
        return call("GET", "/v1/capabilities")

    @mcp.tool(annotations=read_only)
    def robot_state() -> dict[str, object]:
        """Get measured joints, TCP coordinates, gripper and current robot status."""
        return call("GET", "/v1/state")

    @mcp.tool(annotations=read_only, structured_output=False)
    def capture_camera(camera: CameraName) -> list[TextContent | ImageContent]:
        """Capture a FRESH overhead/wrist image with verified broker SHA-256 evidence."""
        response = call("POST", "/v1/capture", {"camera": camera})
        metadata, encoded = _verified_capture(response)
        return [
            TextContent(type="text", text=json.dumps(metadata, sort_keys=True)),
            ImageContent(type="image", data=encoded, mime_type="image/jpeg"),
        ]

    @mcp.tool(annotations=action)
    def go_pose(name: str) -> dict[str, object]:
        """Move to an existing agent-approved saved pose (agent_* names only)."""
        return call("POST", "/v1/go-pose", {"name": name})

    @mcp.tool(annotations=action)
    def jog_joint(joint: JointName, delta_deg: float) -> dict[str, object]:
        """Adjust one named joint by a bounded relative angle in degrees."""
        return call("POST", "/v1/joint", {"joint": joint, "delta_deg": _finite(delta_deg, "delta_deg")})

    @mcp.tool(annotations=action)
    def jog_cartesian(
        frame: Frame,
        x_mm: float = 0.0,
        y_mm: float = 0.0,
        z_mm: float = 0.0,
    ) -> dict[str, object]:
        """Jog TCP translation in model/world or rotating tool coordinates (millimeters).

        Check robot_capabilities for the measured physical world-direction mapping;
        model +Z must NOT be assumed to mean physically up.
        """
        return call("POST", "/v1/jog", {
            "frame": frame,
            "x_mm": _finite(x_mm, "x_mm"),
            "y_mm": _finite(y_mm, "y_mm"),
            "z_mm": _finite(z_mm, "z_mm"),
        })

    @mcp.tool(annotations=action)
    def move_gripper(target: GripperTarget) -> dict[str, object]:
        """Open or close the stock gripper within calibrated endpoint margins."""
        return call("POST", "/v1/gripper", {"target": target})

    @mcp.tool(annotations=action)
    def sleep() -> dict[str, object]:
        """Move to the SDK's validated calibration-relative Sleep posture."""
        return call("POST", "/v1/sleep", {})

    @mcp.tool(annotations=action)
    def sleep_up() -> dict[str, object]:
        """Move to the explicitly requested historical wrist-up Sleep posture."""
        return call("POST", "/v1/sleep-up", {})

    @mcp.tool(annotations=action)
    def stop() -> dict[str, object]:
        """Request bounded STOP/HOLD. Available without a motion lease; not a hardware E-stop."""
        return call("POST", "/v1/stop", {})

    return mcp


def main() -> None:
    """Start a local MCP stdio server; never write non-protocol output to stdout."""
    if not os.environ.get("SOARM101_BROKER_TOKEN", "").strip():
        raise SystemExit("SOARM101_BROKER_TOKEN must be set for the MCP adapter")
    create_server().run(transport="stdio")


if __name__ == "__main__":
    main()
