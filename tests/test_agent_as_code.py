from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "agent-as-code" / "capture_observation.py"


def _write_workstation(path: Path) -> None:
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "follower": {
                    "port": "/dev/ttyACM0",
                    "robot_id": "so101",
                    "calibration": str(path.parent / "so101.json"),
                },
                "leader": {
                    "port": "/dev/ttyACM1",
                    "robot_id": "so101-leader",
                    "calibration": str(path.parent / "so101-leader.json"),
                },
                "selected_camera": "overhead",
                "cameras": {
                    "overhead": {
                        "device": "/dev/video0",
                        "width": 1280,
                        "height": 720,
                        "fps": 30.0,
                        "fourcc": "MJPG",
                        "mirror": False,
                        "auto_start": False,
                        "snapshot_dir": str(path.parent / "captures"),
                    },
                    "wrist": {
                        "device": "/dev/video2",
                        "width": 640,
                        "height": 480,
                        "fps": 30.0,
                        "fourcc": "MJPG",
                        "mirror": False,
                        "auto_start": False,
                        "snapshot_dir": str(path.parent / "captures"),
                    },
                },
            }
        ),
        encoding="utf-8",
    )


def test_agent_as_code_setup_check_uses_named_workstation_cameras(tmp_path: Path) -> None:
    setup = tmp_path / "setup.json"
    setup.write_text(json.dumps({"camera_names": ["wrist", "overhead"]}), encoding="utf-8")
    workstation = tmp_path / "workstation.json"
    _write_workstation(workstation)
    env = dict(os.environ)
    env["SOARM101_WORKSTATION_CONFIG"] = str(workstation)

    completed = subprocess.run(
        [sys.executable, str(SCRIPT), "--setup", str(setup), "--check"],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )

    payload = json.loads(completed.stdout)
    assert payload["valid"] is True
    assert payload["cameras"] == ["wrist", "overhead"]
    assert payload["selected_camera"] == "overhead"
    assert payload["follower"]["port"] == "/dev/ttyACM0"


def test_agent_as_code_setup_defaults_to_all_workstation_cameras(tmp_path: Path) -> None:
    setup = tmp_path / "setup.json"
    setup.write_text(json.dumps({"camera_names": []}), encoding="utf-8")
    workstation = tmp_path / "workstation.json"
    _write_workstation(workstation)
    env = dict(os.environ)
    env["SOARM101_WORKSTATION_CONFIG"] = str(workstation)

    completed = subprocess.run(
        [sys.executable, str(SCRIPT), "--setup", str(setup), "--check"],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )

    payload = json.loads(completed.stdout)
    assert payload["cameras"] == ["overhead", "wrist"]


def test_agent_as_code_setup_rejects_unknown_camera_name(tmp_path: Path) -> None:
    setup = tmp_path / "setup.json"
    setup.write_text(json.dumps({"camera_names": ["missing"]}), encoding="utf-8")
    workstation = tmp_path / "workstation.json"
    _write_workstation(workstation)
    env = dict(os.environ)
    env["SOARM101_WORKSTATION_CONFIG"] = str(workstation)

    completed = subprocess.run(
        [sys.executable, str(SCRIPT), "--setup", str(setup), "--check"],
        capture_output=True,
        text=True,
        env=env,
    )

    assert completed.returncode != 0
    assert "unknown workstation camera profile" in completed.stderr
