from __future__ import annotations

import json

import pytest

from soarm101_motion.camera import CameraSettings, CameraSettingsStore
from soarm101_motion.cli.main import build_parser


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
