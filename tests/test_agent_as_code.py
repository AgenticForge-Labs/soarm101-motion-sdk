from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "agent-as-code" / "capture_observation.py"


def _setup(*, primary_count: int = 1) -> dict[str, object]:
    return {
        "cameras": [
            {
                "name": "side",
                "primary": primary_count > 1,
                "enabled": True,
                "device": "/dev/video2",
                "width": 640,
                "height": 480,
                "fps": 30,
                "fourcc": "MJPG",
                "mirror": False,
            },
            {
                "name": "primary",
                "primary": primary_count >= 1,
                "enabled": True,
                "device": "/dev/video0",
                "width": 1280,
                "height": 720,
                "fps": 30,
                "fourcc": "MJPG",
                "mirror": False,
            },
            {
                "name": "disabled",
                "primary": False,
                "enabled": False,
                "device": "/dev/video4",
                "width": 640,
                "height": 480,
                "fps": 30,
                "fourcc": "MJPG",
                "mirror": False,
            },
        ]
    }


def test_agent_as_code_setup_check_orders_primary_first(tmp_path: Path) -> None:
    setup = tmp_path / "setup.json"
    setup.write_text(json.dumps(_setup()), encoding="utf-8")

    completed = subprocess.run(
        [sys.executable, str(SCRIPT), "--setup", str(setup), "--check"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    assert payload["valid"] is True
    assert payload["primary"] == "primary"
    assert [camera["name"] for camera in payload["cameras"]] == ["primary", "side"]


def test_agent_as_code_setup_rejects_multiple_primary_cameras(tmp_path: Path) -> None:
    setup = tmp_path / "setup.json"
    setup.write_text(json.dumps(_setup(primary_count=2)), encoding="utf-8")

    completed = subprocess.run(
        [sys.executable, str(SCRIPT), "--setup", str(setup), "--check"],
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert "exactly one enabled camera must have primary=true" in completed.stderr
