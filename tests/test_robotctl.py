from __future__ import annotations

import base64
import hashlib
import json
from pathlib import Path

from soarm101_motion import robotctl


def test_robotctl_joint_builds_expected_request(monkeypatch, capsys) -> None:
    observed: dict[str, object] = {}

    def fake_request(*, method: str, path: str, payload=None):
        observed.update({"method": method, "path": path, "payload": payload})
        return {"ok": True, "request_id": "r1", "result": {"completed": True}}

    monkeypatch.setattr(robotctl, "_request", fake_request)
    assert robotctl.main(["joint", "shoulder_pan", "--delta-deg", "5"]) == 0
    assert observed == {
        "method": "POST",
        "path": "/v1/joint",
        "payload": {"joint": "shoulder_pan", "delta_deg": 5.0},
    }
    assert json.loads(capsys.readouterr().out)["ok"] is True


def test_robotctl_capture_writes_sandbox_image(tmp_path: Path, monkeypatch, capsys) -> None:
    image = b"jpeg"
    response = {
        "ok": True,
        "request_id": "r2",
        "result": {
            "name": "overhead",
            "sha256": hashlib.sha256(image).hexdigest(),
            "image_base64": base64.b64encode(image).decode("ascii"),
        },
    }
    monkeypatch.setattr(robotctl, "_request", lambda **kwargs: response)

    output = tmp_path / "observations" / "overhead.jpg"
    assert robotctl.main(["capture", "overhead", "--output", str(output)]) == 0
    assert output.read_bytes() == image
    payload = json.loads(capsys.readouterr().out)
    assert payload["result"]["sandbox_path"] == str(output)
    assert "image_base64" not in payload["result"]


def test_robotctl_has_no_arm_relax_or_raw_command_surface() -> None:
    parser = robotctl.build_parser()
    choices = parser._subparsers._group_actions[0].choices
    assert "arm" not in choices
    assert "disarm" not in choices
    assert "relax" not in choices
    assert "exec" not in choices
    assert "shell" not in choices


def test_robotctl_capture_rejects_sha_mismatch(tmp_path: Path, monkeypatch, capsys) -> None:
    image = b"jpeg"
    response = {
        "ok": True,
        "request_id": "r3",
        "result": {
            "name": "overhead",
            "sha256": hashlib.sha256(b"different").hexdigest(),
            "image_base64": base64.b64encode(image).decode("ascii"),
        },
    }
    monkeypatch.setattr(robotctl, "_request", lambda **kwargs: response)

    output = tmp_path / "overhead.jpg"
    assert robotctl.main(["capture", "overhead", "--output", str(output)]) == 1
    assert not output.exists()
    assert "does not match trusted broker metadata" in capsys.readouterr().err


def test_robotctl_default_capture_names_include_request_id(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    image = b"jpeg"
    responses = iter(
        [
            {
                "ok": True,
                "request_id": "aaaaaaaaaaaa1111",
                "result": {
                    "name": "overhead",
                    "sha256": hashlib.sha256(image).hexdigest(),
                    "image_base64": base64.b64encode(image).decode("ascii"),
                },
            },
            {
                "ok": True,
                "request_id": "bbbbbbbbbbbb2222",
                "result": {
                    "name": "overhead",
                    "sha256": hashlib.sha256(image).hexdigest(),
                    "image_base64": base64.b64encode(image).decode("ascii"),
                },
            },
        ]
    )
    monkeypatch.setattr(robotctl, "_request", lambda **kwargs: next(responses))
    observation_dir = tmp_path / "observations"

    assert robotctl.main(
        ["capture", "overhead", "--observation-dir", str(observation_dir)]
    ) == 0
    first = json.loads(capsys.readouterr().out)["result"]["sandbox_path"]
    assert robotctl.main(
        ["capture", "overhead", "--observation-dir", str(observation_dir)]
    ) == 0
    second = json.loads(capsys.readouterr().out)["result"]["sandbox_path"]

    assert first != second
    assert "aaaaaaaaaaaa" in first
    assert "bbbbbbbbbbbb" in second
    assert Path(first).is_file()
    assert Path(second).is_file()
