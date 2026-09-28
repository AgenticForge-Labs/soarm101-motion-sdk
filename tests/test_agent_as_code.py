from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
AGENT_DIR = ROOT / "agent-as-code"
CAPTURE_SCRIPT = AGENT_DIR / "capture_observation.py"
SETUP_EXAMPLE = AGENT_DIR / "setup.example.json"

sys.path.insert(0, str(AGENT_DIR))
_spec = importlib.util.spec_from_file_location("agent_as_code_host_executor", AGENT_DIR / "host_executor.py")
assert _spec is not None and _spec.loader is not None
host_executor = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = host_executor
_spec.loader.exec_module(host_executor)


def _setup(*, primary_count: int = 1) -> dict[str, object]:
    return {
        "robot": {"port": "/dev/ttyACM0", "robot_id": "so101", "calibration": None},
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
        ],
        "executor": {
            "max_jog_mm": 10,
            "max_jog_deg": 5,
            "max_speed_mm_s": 10,
            "max_acceleration_mm_s2": 40,
        },
    }


def test_agent_as_code_setup_check_orders_primary_first(tmp_path: Path) -> None:
    setup = tmp_path / "setup.json"
    setup.write_text(json.dumps(_setup()), encoding="utf-8")

    completed = subprocess.run(
        [sys.executable, str(CAPTURE_SCRIPT), "--setup", str(setup), "--check"],
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
        [sys.executable, str(CAPTURE_SCRIPT), "--setup", str(setup), "--check"],
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert "exactly one enabled camera must have primary=true" in completed.stderr


def test_executor_accepts_small_jog() -> None:
    limits = host_executor.Limits()
    result = host_executor.validate_jog(
        {"x_mm": 3, "y_mm": 4, "speed_mm_s": 5, "acceleration_mm_s2": 20},
        limits,
    )
    assert result["x_mm"] == 3
    assert result["y_mm"] == 4
    assert result["frame"] == "world"


def test_executor_rejects_jog_outside_outer_envelope() -> None:
    with pytest.raises(host_executor.RequestError, match="exceeds executor limit"):
        host_executor.validate_jog({"x_mm": 8, "y_mm": 8}, host_executor.Limits())


@pytest.mark.parametrize(
    ("target", "expected"),
    [("open", "open"), ("close", "close"), (0.5, "0.5"), ("1", "1")],
)
def test_executor_validates_gripper(target: object, expected: str) -> None:
    assert host_executor.validate_gripper({"target": target}) == expected


@pytest.mark.parametrize("target", [-0.1, 1.1, "nope"])
def test_executor_rejects_bad_gripper_target(target: object) -> None:
    with pytest.raises(host_executor.RequestError):
        host_executor.validate_gripper({"target": target})


def test_reference_setup_pins_hermes_vision_model_and_limits() -> None:
    setup = json.loads(SETUP_EXAMPLE.read_text(encoding="utf-8"))
    assert setup["agents"]["hermes"]["provider"] == "openrouter"
    assert setup["agents"]["hermes"]["model"] == "deepseek/deepseek-v4.1-flash"
    assert setup["executor"]["port"] == 8765
    assert setup["executor"]["max_jog_mm"] == 10.0


def test_openshell_profiles_are_endpoint_scoped() -> None:
    robot = (AGENT_DIR / "openshell/providers/robot-executor.yaml").read_text(encoding="utf-8")
    assert "host: host.openshell.internal" in robot
    assert "port: 8765" in robot
    assert "/v1/jog" in robot
    assert "/dev/tty" not in robot

    hermes = (AGENT_DIR / "openshell/providers/hermes-openrouter.yaml").read_text(encoding="utf-8")
    assert "host: openrouter.ai" in hermes
    assert "/opt/hermes/.venv/bin/python" in hermes


def test_agent_as_code_shell_scripts_parse() -> None:
    scripts = [
        AGENT_DIR / "install.sh",
        AGENT_DIR / "doctor.sh",
        AGENT_DIR / "run-codex.sh",
        AGENT_DIR / "run-hermes.sh",
        AGENT_DIR / "openshell/codex/run-agent.sh",
        AGENT_DIR / "openshell/hermes/run-agent.sh",
    ]
    for script in scripts:
        subprocess.run(["bash", "-n", str(script)], check=True)
        assert os.access(script, os.X_OK), f"{script} should be executable"


def test_observation_ids_are_unique_within_one_second() -> None:
    first = __import__("capture_observation")._observation_id("same", 1_700_000_000_000_000_001)
    second = __import__("capture_observation")._observation_id("same", 1_700_000_000_000_000_002)
    assert first != second
    assert first.endswith("-same")


def test_executor_rejects_nonstandard_provider_port(tmp_path: Path) -> None:
    setup = _setup()
    setup["executor"]["port"] = 9999  # type: ignore[index]
    path = tmp_path / "setup.json"
    path.write_text(json.dumps(setup), encoding="utf-8")
    with pytest.raises(ValueError, match="must currently be 8765"):
        host_executor.load_setup(path)


def test_executor_serializes_cli_access(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    setup = _setup()
    setup["executor"]["port"] = 8765  # type: ignore[index]
    path = tmp_path / "setup.json"
    path.write_text(json.dumps(setup), encoding="utf-8")
    executor = host_executor.RobotExecutor(path, "x" * 48)

    active = 0
    peak = 0
    guard = threading.Lock()

    class Completed:
        returncode = 0
        stdout = "{}"
        stderr = ""

    def fake_run(*args: object, **kwargs: object) -> Completed:
        nonlocal active, peak
        with guard:
            active += 1
            peak = max(peak, active)
        time.sleep(0.03)
        with guard:
            active -= 1
        return Completed()

    monkeypatch.setattr(host_executor.subprocess, "run", fake_run)

    workers = [
        threading.Thread(target=executor.state),
        threading.Thread(target=executor.diagnose),
    ]
    for worker in workers:
        worker.start()
    for worker in workers:
        worker.join()

    assert peak == 1
