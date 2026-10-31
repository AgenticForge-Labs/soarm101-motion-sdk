"""Broker remains independent of the optional, future Puppeteer-owned MCP."""

from __future__ import annotations

import ast
from pathlib import Path

from soarm101_motion.capability_profile import (
    BROKER_CAPABILITY_ROUTES,
    ROUTE_TO_TOOL,
    TOOL_ROUTES,
    CapabilityProfile,
)


def test_broker_route_map_is_the_canonical_mcp_independent_registry() -> None:
    assert TOOL_ROUTES is BROKER_CAPABILITY_ROUTES
    assert len(BROKER_CAPABILITY_ROUTES) == 14
    assert len(set(BROKER_CAPABILITY_ROUTES.values())) == 14
    assert ROUTE_TO_TOOL[("POST", "/v1/sleep_up")] == "sleep_up"
    assert {
        "robot_health", "robot_capabilities", "robot_state", "list_assets",
        "replay_recording", "replay_sequence", "capture_camera", "go_pose",
        "jog_joint", "jog_cartesian", "move_gripper", "sleep", "sleep_up", "stop",
    }.issubset(BROKER_CAPABILITY_ROUTES)
    assert {"list_assets", "replay_recording", "replay_sequence"}.isdisjoint(
        CapabilityProfile.full().allowed_tools
    )


def test_broker_profile_is_not_implemented_by_mcp() -> None:
    """Broker/profile import does not load Puppeteer or any MCP server module."""
    module = Path(__file__).resolve().parents[1] / "src/soarm101_motion/capability_profile.py"
    parsed = ast.parse(module.read_text(encoding="utf-8"))
    imports = []
    for node in ast.walk(parsed):
        if isinstance(node, ast.Import):
            imports.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imports.append(node.module or "")
    assert not any(
        name == "mcp" or name.startswith("mcp.") or "puppeteer" in name
        for name in imports
    )


def test_profile_enforcement_does_not_depend_on_an_mcp_server() -> None:
    profile = CapabilityProfile.from_document({
        "schema_version": 1,
        "name": "status-only",
        "tools": ["robot_health", "robot_capabilities", "robot_state"],
        "cameras": [],
        "limits": {},
    })
    profile.check("GET", "/v1/state", {})
    profile.check("GET", "/v1/profile", {})
    import pytest

    with pytest.raises(PermissionError, match="not enabled"):
        profile.check("POST", "/v1/replay-recording", {
            "asset_id": "unapproved",
            "revision": "sha256:" + "a" * 64,
            "session_id": "unknown",
            "request_id": "unknown",
        })
    with pytest.raises(PermissionError, match="not enabled"):
        profile.check("POST", "/v1/joint", {"joint": "shoulder_pan", "delta_deg": 1})


def test_native_sdk_package_does_not_own_mcp_server() -> None:
    """Native Python/CLI/GUI/broker stay installable without Puppet or MCP."""
    root = Path(__file__).resolve().parents[1]
    project = (root / "pyproject.toml").read_text(encoding="utf-8")
    assert 'soarm101-mcp =' not in project
    assert 'mcp = ["mcp' not in project
    assert not (root / "src/soarm101_motion/mcp_server.py").exists()
    assert not (root / "src/soarm101_motion/mcp_guidance.py").exists()
    assert not (root / "src/soarm101_motion/agent_mcp_runtime.py").exists()
    native = (root / "src/soarm101_motion/agent_sandbox.py").read_text()
    assert "_puppeteer_mcp()" in native
    assert "from .agent_mcp_runtime" not in native
