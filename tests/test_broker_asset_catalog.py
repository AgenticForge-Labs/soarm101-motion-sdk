import asyncio

import pytest

from soarm101_motion.broker import AgentCommandExecutor, BrokerRelayService, RobotBrokerService
from soarm101_motion.capability_profile import CapabilityProfile
from soarm101_motion.mcp_server import create_server
from test_asset_catalog import libraries


def profile(*tools):
    return CapabilityProfile.from_document({
        "schema_version": 1, "name": "catalog-test", "tools": list(tools),
        "cameras": [], "limits": {},
    })


def test_catalog_requires_explicit_grant_and_paths(tmp_path):
    poses, trajectories, sequences = libraries(tmp_path)
    paths = (poses.path, trajectories.root, sequences.path)
    denied = RobotBrokerService(token="token", event_path=tmp_path / "events")
    assert denied.dispatch("POST", "/v1/assets", {}).status == 403
    assert "list_assets" not in CapabilityProfile.full().allowed_tools
    with pytest.raises(ValueError, match="explicit profile"):
        RobotBrokerService(token="token", catalog_paths=paths)
    with pytest.raises(ValueError, match="requires operator-selected"):
        RobotBrokerService(token="token", profile=profile("list_assets"))


def test_catalog_filter_pagination_and_relay_enforcement(tmp_path):
    poses, trajectories, sequences = libraries(tmp_path)
    service = RobotBrokerService(
        executor=AgentCommandExecutor(robot_id="arm"), token="token",
        event_path=tmp_path / "events", profile=profile("list_assets"),
        catalog_paths=(poses.path, trajectories.root, sequences.path),
    )
    result = service.dispatch("POST", "/v1/assets", {"limit": 1})
    assert result.status == 200
    assert len(result.body["result"]["items"]) == 1
    assert result.body["result"]["next_offset"] == 1
    assert str(tmp_path) not in str(result.body)
    second = service.dispatch("POST", "/v1/assets", {"kind": "recording"})
    assert second.body["result"]["items"][0]["id"] == "soarm101/recording/raw/wave"
    assert service.dispatch("POST", "/v1/assets", {"path": "/tmp", "limit": 1}).status == 400
    assert service.dispatch("POST", "/v1/assets", {"limit": True}).status == 400
    relay = BrokerRelayService(backend=service, token="relay", agent_profile=profile())
    assert relay.dispatch("POST", "/v1/assets", {}).status == 403


def test_catalog_tool_only_appears_when_granted():
    pytest.importorskip("mcp")
    from mcp import Client

    calls = []
    def request(*, method, path, payload):
        calls.append((method, path, payload))
        return {"ok": True, "result": {"items": [], "next_offset": None}}

    async def run():
        async with Client(create_server(request, allowed_tools={"list_assets"})) as client:
            assert {tool.name for tool in (await client.list_tools()).tools} == {"list_assets"}
            result = await client.call_tool("list_assets", {"limit": 2, "kind": "pose"})
            assert not result.is_error
    asyncio.run(run())
    assert calls == [("POST", "/v1/assets", {"offset": 0, "limit": 2, "kind": "pose"})]
