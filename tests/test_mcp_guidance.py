"""MCP guidance resources/prompts are semantic-only and hardware-free."""

from __future__ import annotations

import asyncio

import pytest

from soarm101_motion.mcp_guidance import GUIDANCE_RESOURCES, task_prompt
from soarm101_motion.mcp_server import create_server


def test_guidance_is_static_and_preserves_authority_boundary() -> None:
    core = GUIDANCE_RESOURCES["core"]
    assert "robot_capabilities" in core
    assert "robot_state" in core
    assert "observe -> decide -> smallest useful bounded action -> re-observe" in core
    assert "model +Z is not automatically" in core
    assert "does not grant" in core
    recovery = GUIDANCE_RESOURCES["recovery"]
    assert "human must re-arm" in recovery
    assert "do not route around" in recovery
    assert "alternate path" in recovery


@pytest.mark.parametrize("strategy", ["auto", "coordinates", "joints", "observe-only"])
def test_task_prompt_supports_experimental_strategy(strategy: str) -> None:
    prompt = task_prompt("place the block in the marked area", strategy)
    assert "place the block in the marked area" in prompt
    assert "robot_capabilities" in prompt
    assert "robot_state" in prompt
    assert "fresh" in prompt
    assert "arm the robot" not in prompt.lower()


@pytest.mark.parametrize("task,strategy", [("", "auto"), ("test", "raw-servo")])
def test_task_prompt_fails_closed_on_invalid_inputs(task: str, strategy: str) -> None:
    with pytest.raises(ValueError):
        task_prompt(task, strategy)


def test_mcp_lists_and_reads_guidance_and_prompt() -> None:
    pytest.importorskip("mcp")
    from mcp import Client

    server = create_server(
        request_fn=lambda **_: {"ok": True, "request_id": "x", "result": {}},
        allowed_tools={"robot_health", "robot_capabilities", "robot_state"},
    )

    async def run() -> None:
        async with Client(server) as client:
            resources = await client.list_resources()
            uris = {str(item.uri) for item in resources.resources}
            assert uris == {f"soarm101://guidance/{name}" for name in GUIDANCE_RESOURCES}
            core = await client.read_resource("soarm101://guidance/core")
            assert "world frame" in str(core.contents[0]).lower()
            prompts = await client.list_prompts()
            assert {item.name for item in prompts.prompts} == {"operate_robot_task"}
            prompt = await client.get_prompt(
                "operate_robot_task",
                {"task": "inspect the workspace", "strategy": "observe-only"},
            )
            rendered = str(prompt.messages)
            assert "inspect the workspace" in rendered
            assert "Do not request physical motion" in rendered
            tools = {item.name for item in (await client.list_tools()).tools}
            assert tools == {"robot_health", "robot_capabilities", "robot_state"}

    asyncio.run(run())
