from __future__ import annotations

import base64
import hashlib
import json
import threading
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from soarm101_motion import SOARM101Config
from soarm101_motion.broker import (
    AgentCommandExecutor,
    AgentMotionRates,
    BrokerCommandError,
    RobotBrokerHTTPServer,
    RobotBrokerService,
)





def test_broker_executor_propagates_trusted_motion_envelope(monkeypatch) -> None:
    commands: list[list[str]] = []

    class Completed:
        returncode = 0
        stdout = "{}"
        stderr = ""

    def fake_run(command, **kwargs):
        del kwargs
        commands.append(list(command))
        return Completed()

    monkeypatch.setattr("soarm101_motion.broker.subprocess.run", fake_run)
    config = SOARM101Config.from_motion_limits(
        robot_id="so101",
        max_joint_speed_deg_s=80.0,
        max_joint_acceleration_deg_s2=500.0,
        max_linear_speed_mm_s=90.0,
        max_linear_acceleration_mm_s2=900.0,
        max_tool_angular_speed_deg_s=70.0,
        max_tool_angular_acceleration_deg_s2=700.0,
    )
    executor = AgentCommandExecutor(config=config)

    executor.run(["state", "--robot-id", "so101"])
    motion_command = commands[-1]
    assert motion_command[-12:] == [
        "--max-joint-speed-deg-s",
        "8",
        "--max-joint-acceleration-deg-s2",
        "25",
        "--max-linear-speed-mm-s",
        "10",
        "--max-linear-acceleration-mm-s2",
        "40",
        "--max-tool-angular-speed-deg-s",
        "70",
        "--max-tool-angular-acceleration-deg-s2",
        "700",
    ]

    executor.run(["capture", "overhead"])
    capture_command = commands[-1]
    assert "--max-joint-speed-deg-s" not in capture_command
    assert "--max-linear-speed-mm-s" not in capture_command


def test_broker_pins_global_rates_and_allows_only_slower_requests(monkeypatch) -> None:
    commands: list[list[str]] = []

    class Completed:
        returncode = 0
        stdout = "{}"
        stderr = ""

    def fake_run(command, **kwargs):
        del kwargs
        commands.append(list(command))
        return Completed()

    monkeypatch.setattr("soarm101_motion.broker.subprocess.run", fake_run)
    config = SOARM101Config.from_motion_limits(
        robot_id="so101",
        max_joint_speed_deg_s=60.0,
        max_joint_acceleration_deg_s2=300.0,
        max_linear_speed_mm_s=80.0,
        max_linear_acceleration_mm_s2=500.0,
    )
    rates = AgentMotionRates(
        joint_speed_deg_s=16.0,
        joint_acceleration_deg_s2=50.0,
        cartesian_speed_mm_s=20.0,
        cartesian_acceleration_mm_s2=80.0,
    )
    executor = AgentCommandExecutor(config=config, rates=rates)
    executor.run(["jog", "--frame", "world", "--x-mm", "5"])
    command = commands[-1]
    assert command[command.index("--speed-mm-s") + 1] == "20"
    assert command[command.index("--acceleration-mm-s2") + 1] == "80"
    assert command[command.index("--max-linear-speed-mm-s") + 1] == "20"

    executor.run(["joint", "shoulder_pan", "--delta-deg", "3"])
    command = commands[-1]
    assert command[command.index("--speed-deg-s") + 1] == "16"
    assert command[command.index("--acceleration-deg-s2") + 1] == "50"
    assert command[command.index("--max-joint-speed-deg-s") + 1] == "16"

    executor.run(["capture", "overhead"])
    assert "--speed-mm-s" not in commands[-1]
    assert "--speed-deg-s" not in commands[-1]


@pytest.mark.parametrize("field,value", [
    ("joint_speed_deg_s", -1.0),
    ("joint_speed_deg_s", float("nan")),
    ("cartesian_speed_mm_s", 0.0),
    ("cartesian_acceleration_mm_s2", float("inf")),
    ("cartesian_speed_mm_s", 101.0),
    ("joint_acceleration_deg_s2", 1001.0),
])
def test_broker_rejects_invalid_motion_requests_before_startup(field, value) -> None:
    rates = AgentMotionRates(**{field: value})
    with pytest.raises(ValueError, match=field):
        AgentCommandExecutor(rates=rates)


def test_broker_parser_defaults_to_100_1000_motion_envelope() -> None:
    from soarm101_motion.broker import build_parser

    args = build_parser().parse_args([])
    assert args.max_joint_speed_deg_s == pytest.approx(100.0)
    assert args.max_joint_acceleration_deg_s2 == pytest.approx(1000.0)
    assert args.max_linear_speed_mm_s == pytest.approx(100.0)
    assert args.max_linear_acceleration_mm_s2 == pytest.approx(1000.0)
    assert args.max_tool_angular_speed_deg_s == pytest.approx(100.0)
    assert args.max_tool_angular_acceleration_deg_s2 == pytest.approx(1000.0)
    assert args.agent_joint_speed_deg_s == pytest.approx(8.0)
    assert args.agent_joint_acceleration_deg_s2 == pytest.approx(25.0)
    assert args.agent_cartesian_speed_mm_s == pytest.approx(10.0)
    assert args.agent_cartesian_acceleration_mm_s2 == pytest.approx(40.0)
    assert args.agent_gripper_speed_raw == 250
    assert args.agent_gripper_acceleration_raw == 20


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


@pytest.mark.parametrize("action,path,body,expected_flags", [
    ("go_pose", "/v1/go-pose", {"name": "agent_start_overhead"},
     {"--speed-deg-s": "15", "--acceleration-deg-s2": "150",
      "--gripper-speed-raw": "250", "--gripper-acceleration-raw": "20"}),
    ("sleep", "/v1/sleep", {},
     {"--speed-deg-s": "15", "--acceleration-deg-s2": "150",
      "--gripper-speed-raw": "250", "--gripper-acceleration-raw": "20"}),
    ("sleep_up", "/v1/sleep-up", {},
     {"--speed-deg-s": "15", "--acceleration-deg-s2": "150",
      "--gripper-speed-raw": "250", "--gripper-acceleration-raw": "20"}),
    ("joint", "/v1/joint", {"joint": "shoulder_pan", "delta_deg": 2},
     {"--speed-deg-s": "15", "--acceleration-deg-s2": "150"}),
    ("jog", "/v1/jog", {"frame": "world", "x_mm": 1},
     {"--speed-mm-s": "10", "--acceleration-mm-s2": "40"}),
    ("gripper", "/v1/gripper", {"target": "open"},
     {"--gripper-speed-raw": "250", "--gripper-acceleration-raw": "20"}),
])
def test_every_broker_motion_uses_pinned_rates(
    monkeypatch, tmp_path, action, path, body, expected_flags,
) -> None:
    commands = []

    def fake_run(command, **kwargs):
        commands.append(command)
        class Completed:
            returncode = 0
            stdout = "{}"
            stderr = ""
        return Completed()

    monkeypatch.setattr("soarm101_motion.broker.subprocess.run", fake_run)
    executor = AgentCommandExecutor(rates=AgentMotionRates(
        joint_speed_deg_s=15.0, joint_acceleration_deg_s2=150.0,
    ))
    service = RobotBrokerService(
        executor=executor, token="test-token",
        event_path=tmp_path / "events.jsonl",
    )
    response = service.dispatch("POST", path, body)
    assert response.status == 200, response.body
    assert commands, action
    command = commands[-1]
    for flag, expected in expected_flags.items():
        assert command[command.index(flag) + 1] == expected
    assert command[command.index("--max-joint-speed-deg-s") + 1] == "15"
    assert command[command.index("--max-joint-acceleration-deg-s2") + 1] == "150"


@pytest.mark.parametrize("action,path,body", [
    ("go_pose", "/v1/go-pose", {"name": "agent_start_overhead", "speed_deg_s": 15.01}),
    ("sleep", "/v1/sleep", {"acceleration_deg_s2": 151}),
    ("joint", "/v1/joint", {"joint": "shoulder_pan", "delta_deg": 2,
                             "speed_deg_s": 16}),
    ("jog", "/v1/jog", {"x_mm": 1, "speed_mm_s": 11}),
    ("gripper", "/v1/gripper", {"target": "open", "gripper_speed_raw": 251}),
    ("sleep_up", "/v1/sleep-up", {"gripper_acceleration_raw": 21}),
])
def test_motion_faster_than_human_policy_rejected_without_execution(
    monkeypatch, tmp_path, action, path, body,
) -> None:
    commands = []
    monkeypatch.setattr(
        "soarm101_motion.broker.subprocess.run",
        lambda cmd, **kw: commands.append(cmd),
    )
    service = RobotBrokerService(
        executor=AgentCommandExecutor(rates=AgentMotionRates(
            joint_speed_deg_s=15, joint_acceleration_deg_s2=150,
        )), token="test-token", event_path=tmp_path / "events.jsonl",
    )
    result = service.dispatch("POST", path, body)
    assert result.status == 400, (action, result.body)
    assert not commands


def test_agent_may_request_lower_rates_on_saved_pose_and_sleep(monkeypatch, tmp_path) -> None:
    commands = []
    class Completed:
        returncode = 0
        stdout = "{}"
        stderr = ""
    monkeypatch.setattr(
        "soarm101_motion.broker.subprocess.run",
        lambda cmd, **kw: (commands.append(cmd), Completed())[1],
    )
    service = RobotBrokerService(
        executor=AgentCommandExecutor(rates=AgentMotionRates(
            joint_speed_deg_s=15, joint_acceleration_deg_s2=150,
        )), token="test-token", event_path=tmp_path / "events.jsonl",
    )
    for path, extra in (
        ("/v1/go-pose", {"name": "agent_start_overhead"}),
        ("/v1/sleep", {}),
        ("/v1/sleep-up", {}),
    ):
        body = {**extra, "speed_deg_s": 5, "acceleration_deg_s2": 30,
                "gripper_speed_raw": 100, "gripper_acceleration_raw": 10}
        assert service.dispatch("POST", path, body).status == 200
        cmd = commands[-1]
        for flag, expected in (("--speed-deg-s", "5"),
                               ("--acceleration-deg-s2", "30"),
                               ("--gripper-speed-raw", "100"),
                               ("--gripper-acceleration-raw", "10")):
            assert cmd[cmd.index(flag) + 1] == expected


@pytest.mark.parametrize("value", [0, -1, float("inf"), float("nan"), True])
def test_bad_requested_joint_rates_fail_closed(value) -> None:
    rates = AgentMotionRates(joint_speed_deg_s=15, joint_acceleration_deg_s2=150)
    with pytest.raises(ValueError):
        rates.selected("sleep", {"speed_deg_s": value})
