from __future__ import annotations

import json

import pytest

from soarm101_motion import camera as camera_module
from soarm101_motion import workstation as workstation_module
from soarm101_motion.camera import CameraCapture, CameraSettings, CameraSettingsStore
from soarm101_motion.cli.main import build_parser


def test_camera_defaults_are_low_bandwidth_for_robotics() -> None:
    settings = CameraSettings()
    assert settings.width == 640
    assert settings.height == 480
    assert settings.fps == 15.0
    assert settings.fourcc == "MJPG"


def test_camera_settings_round_trip(tmp_path) -> None:
    path = tmp_path / "camera.json"
    store = CameraSettingsStore(path)
    settings = CameraSettings(
        device="/dev/video7",
        width=1920,
        height=1080,
        fps=30.0,
        fourcc="MJPG",
        mirror=True,
        auto_start=True,
        snapshot_dir=str(tmp_path / "captures"),
    )
    assert store.save(settings) == settings
    assert store.load() == settings
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["device"] == "/dev/video7"
    assert payload["mirror"] is True


def test_camera_settings_validation_rejects_bad_fourcc() -> None:
    with pytest.raises(ValueError, match="fourcc"):
        CameraSettings(fourcc="MJ").validated()


def test_camera_capture_requests_two_opencv_buffers(monkeypatch) -> None:
    class FakeCapture:
        def __init__(self) -> None:
            self.properties: dict[int, float] = {}
            self.released = False

        def isOpened(self) -> bool:
            return True

        def set(self, prop: int, value: float) -> bool:
            self.properties[prop] = value
            return True

        def release(self) -> None:
            self.released = True

    class FakeCV2:
        CAP_V4L2 = 200
        CAP_PROP_FOURCC = 1
        CAP_PROP_FRAME_WIDTH = 2
        CAP_PROP_FRAME_HEIGHT = 3
        CAP_PROP_FPS = 4
        CAP_PROP_BUFFERSIZE = 5

        def __init__(self) -> None:
            self.capture = FakeCapture()

        @staticmethod
        def VideoWriter_fourcc(*_: str) -> int:
            return 0

        def VideoCapture(self, *_: object) -> FakeCapture:
            return self.capture

    fake_cv2 = FakeCV2()
    monkeypatch.setattr(camera_module, "_opencv", lambda: fake_cv2)
    monkeypatch.setattr(camera_module.sys, "platform", "linux")
    monkeypatch.setattr(camera_module, "camera_device_available", lambda _device: True)

    capture = CameraCapture(CameraSettings(device="/dev/video-test")).open()
    assert fake_cv2.capture.properties[FakeCV2.CAP_PROP_BUFFERSIZE] == 2.0
    capture.close()
    assert fake_cv2.capture.released


def test_camera_cli_parser_exposes_basic_agent_tools() -> None:
    parser = build_parser()
    assert parser.parse_args(["camera", "list"]).camera_command == "list"
    assert parser.parse_args(["camera", "show"]).camera_command == "show"
    configured = parser.parse_args(
        [
            "camera",
            "configure",
            "--device",
            "/dev/video2",
            "--width",
            "640",
            "--height",
            "480",
            "--fps",
            "30",
            "--mirror",
        ]
    )
    assert configured.device == "/dev/video2"
    assert configured.width == 640
    assert configured.height == 480
    assert configured.fps == 30.0
    assert configured.mirror is True
    captured = parser.parse_args(
        ["camera", "capture", "--output", "frame.jpg", "--json"]
    )
    assert captured.output == "frame.jpg"
    assert captured.json is True



def test_named_camera_and_workstation_cli_persist_profiles(tmp_path, monkeypatch, capsys) -> None:
    from soarm101_motion.cli.main import main

    workstation = tmp_path / "workstation.json"
    monkeypatch.setenv("SOARM101_WORKSTATION_CONFIG", str(workstation))
    monkeypatch.setattr(
        workstation_module,
        "DEFAULT_CAMERA_CONFIG_PATH",
        tmp_path / "legacy-camera.json",
    )

    assert (
        main(
            [
                "camera",
                "configure",
                "--name",
                "overhead",
                "--device",
                "/dev/video2",
                "--width",
                "1280",
                "--height",
                "720",
                "--fps",
                "30",
                "--json",
            ]
        )
        == 0
    )
    configured = json.loads(capsys.readouterr().out)
    assert configured["name"] == "overhead"
    assert configured["device"] == "/dev/video2"

    assert (
        main(
            [
                "camera",
                "configure",
                "--name",
                "wrist",
                "--device",
                "/dev/video4",
                "--width",
                "640",
                "--height",
                "480",
                "--json",
            ]
        )
        == 0
    )
    json.loads(capsys.readouterr().out)

    assert main(["camera", "show", "--json"]) == 0
    cameras = json.loads(capsys.readouterr().out)
    assert cameras["selected_camera"] == "wrist"
    assert set(cameras["cameras"]) == {"overhead", "wrist"}

    assert (
        main(
            [
                "workstation",
                "arm",
                "follower",
                "--port",
                "/dev/ttyACM0",
                "--robot-id",
                "so101",
                "--json",
            ]
        )
        == 0
    )
    follower = json.loads(capsys.readouterr().out)
    assert follower["port"] == "/dev/ttyACM0"

    assert main(["workstation", "show", "--json"]) == 0
    profile = json.loads(capsys.readouterr().out)
    assert profile["follower"]["port"] == "/dev/ttyACM0"
    assert set(profile["cameras"]) == {"overhead", "wrist"}


def test_camera_cli_parser_exposes_named_and_all_capture() -> None:
    parser = build_parser()
    named = parser.parse_args(["camera", "capture", "--name", "overhead", "--json"])
    assert named.name == "overhead"
    assert named.all is False
    every = parser.parse_args(["camera", "capture", "--all", "--json"])
    assert every.all is True
