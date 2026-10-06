from __future__ import annotations

import base64
import hashlib
import json
import threading
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from soarm101_motion.broker import (
    BrokerCommandError,
    RobotBrokerHTTPServer,
    RobotBrokerService,
)


class FakeExecutor:
    def __init__(self, responses: dict[tuple[str, ...], dict[str, object]]) -> None:
        self.robot_id = "so101"
        self.responses = responses
        self.calls: list[tuple[str, ...]] = []

    def run(self, arguments: list[str] | tuple[str, ...]) -> dict[str, object]:
        key = tuple(arguments)
        self.calls.append(key)
        response = self.responses.get(key)
        if response is None:
            raise BrokerCommandError(f"unexpected command: {key}")
        return dict(response)


def test_broker_exposes_only_bounded_agent_routes(tmp_path: Path) -> None:
    executor = FakeExecutor(
        {
            ("capabilities", "--robot-id", "so101"): {"actions": {"stop": "always_available"}},
            (
                "joint",
                "shoulder_pan",
                "--delta-deg",
                "5",
                "--robot-id",
                "so101",
            ): {"completed": True, "holding": True},
        }
    )
    service = RobotBrokerService(
        executor=executor,  # type: ignore[arg-type]
        token="secret",
        event_path=tmp_path / "events.jsonl",
    )

    capabilities = service.dispatch("GET", "/v1/capabilities")
    assert capabilities.status == 200
    assert capabilities.body["ok"] is True

    joint = service.dispatch(
        "POST",
        "/v1/joint",
        {"joint": "shoulder_pan", "delta_deg": 5},
    )
    assert joint.status == 200
    assert joint.body["ok"] is True

    assert service.dispatch("POST", "/v1/arm", {}).status == 404
    assert service.dispatch("POST", "/v1/disarm", {}).status == 404
    assert service.dispatch("POST", "/v1/relax", {}).status == 404


def test_broker_requires_exact_bearer_token(tmp_path: Path) -> None:
    service = RobotBrokerService(
        executor=FakeExecutor({}),  # type: ignore[arg-type]
        token="secret",
        event_path=tmp_path / "events.jsonl",
    )
    assert service.authorized(None) is False
    assert service.authorized("Bearer wrong") is False
    assert service.authorized("Bearer secret") is True


def test_broker_capture_returns_image_bytes_and_sha(tmp_path: Path) -> None:
    image_path = tmp_path / "capture.jpg"
    image_bytes = b"fake-jpeg-bytes"
    image_path.write_bytes(image_bytes)
    executor = FakeExecutor(
        {
            ("capture", "overhead"): {
                "name": "overhead",
                "path": str(image_path),
                "timestamp": 1.0,
                "width": 10,
                "height": 10,
                "device": "/dev/v4l/by-id/test-camera-video-index0",
            }
        }
    )
    service = RobotBrokerService(
        executor=executor,  # type: ignore[arg-type]
        token="secret",
        event_path=tmp_path / "events.jsonl",
    )

    response = service.dispatch("POST", "/v1/capture", {"camera": "overhead"})
    assert response.status == 200
    result = response.body["result"]
    assert isinstance(result, dict)
    assert base64.b64decode(result["image_base64"]) == image_bytes
    assert result["sha256"] == hashlib.sha256(image_bytes).hexdigest()
    assert "path" not in result
    assert "device" not in result

    events = [
        json.loads(line)
        for line in (tmp_path / "events.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert events[0]["action"] == "capture"
    assert events[0]["ok"] is True
    assert events[0]["result"]["device"] == "/dev/v4l/by-id/test-camera-video-index0"
    assert "image_base64" not in events[0]["result"]
    assert events[1]["action"] == "capture_evidence"
    assert events[1]["request_id"] == events[0]["request_id"]
    assert events[1]["result"]["sha256"] == hashlib.sha256(image_bytes).hexdigest()
    assert events[1]["result"]["host_path"] == str(image_path)
    assert events[1]["result"]["device"] == "/dev/v4l/by-id/test-camera-video-index0"
    assert "image_base64" not in events[1]["result"]


def test_broker_translates_tool_frame_jog_to_bounded_cli(tmp_path: Path) -> None:
    expected = (
        "jog",
        "--frame",
        "tool",
        "--x-mm",
        "1",
        "--y-mm",
        "-2",
        "--z-mm",
        "3",
        "--robot-id",
        "so101",
    )
    executor = FakeExecutor({expected: {"completed": True, "holding": True}})
    service = RobotBrokerService(
        executor=executor,  # type: ignore[arg-type]
        token="secret",
        event_path=tmp_path / "events.jsonl",
    )

    response = service.dispatch(
        "POST",
        "/v1/jog",
        {"frame": "tool", "x_mm": 1, "y_mm": -2, "z_mm": 3},
    )
    assert response.status == 200
    assert executor.calls == [expected]


@pytest.mark.parametrize("frame", ["camera", "base_link", ""])
def test_broker_rejects_unknown_jog_frame(tmp_path: Path, frame: str) -> None:
    service = RobotBrokerService(
        executor=FakeExecutor({}),  # type: ignore[arg-type]
        token="secret",
        event_path=tmp_path / "events.jsonl",
    )
    response = service.dispatch(
        "POST",
        "/v1/jog",
        {"frame": frame, "x_mm": 1, "y_mm": 0, "z_mm": 0},
    )
    assert response.status == 400
    assert response.body["ok"] is False


def test_broker_capture_failure_does_not_record_success_evidence(tmp_path: Path) -> None:
    missing = tmp_path / "missing.jpg"
    executor = FakeExecutor(
        {
            ("capture", "overhead"): {
                "name": "overhead",
                "path": str(missing),
                "timestamp": 1.0,
                "width": 10,
                "height": 10,
            }
        }
    )
    events_path = tmp_path / "events.jsonl"
    service = RobotBrokerService(
        executor=executor,  # type: ignore[arg-type]
        token="secret",
        event_path=events_path,
    )

    response = service.dispatch("POST", "/v1/capture", {"camera": "overhead"})
    assert response.status == 409
    events = [
        json.loads(line)
        for line in events_path.read_text(encoding="utf-8").splitlines()
    ]
    assert len(events) == 1
    assert events[0]["action"] == "capture"
    assert events[0]["ok"] is False
    assert not any(row["action"] == "capture_evidence" for row in events)


def test_http_server_enforces_token_and_routes_health(tmp_path: Path) -> None:
    service = RobotBrokerService(
        executor=FakeExecutor({}),  # type: ignore[arg-type]
        token="secret",
        event_path=tmp_path / "events.jsonl",
    )
    server = RobotBrokerHTTPServer(("127.0.0.1", 0), service)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        host, port = server.server_address
        url = f"http://{host}:{port}/v1/health"

        with pytest.raises(urllib.error.HTTPError) as exc_info:
            urllib.request.urlopen(url, timeout=2)
        assert exc_info.value.code == 401

        request = urllib.request.Request(
            url,
            headers={"Authorization": "Bearer secret"},
        )
        with urllib.request.urlopen(request, timeout=2) as response:
            payload = json.loads(response.read().decode("utf-8"))
        assert response.status == 200
        assert payload["ok"] is True
        assert payload["result"]["service"] == "soarm101-agent-broker"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)

def test_broker_translates_sleep_up_to_bounded_cli(tmp_path: Path) -> None:
    expected = ("sleep-up", "--robot-id", "so101")
    executor = FakeExecutor({expected: {"completed": True, "holding": True}})
    service = RobotBrokerService(
        executor=executor,  # type: ignore[arg-type]
        token="secret",
        event_path=tmp_path / "events.jsonl",
    )

    response = service.dispatch("POST", "/v1/sleep-up", {})

    assert response.status == 200
    assert executor.calls == [expected]
