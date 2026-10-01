from __future__ import annotations

import importlib.util
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest


def _load_example_module():
    root = Path(__file__).resolve().parents[1]
    path = root / "examples" / "paper_workspace_calibration.py"
    spec = importlib.util.spec_from_file_location("paper_workspace_calibration", path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_paper_workspace_calibration_help_runs_without_hardware() -> None:
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [sys.executable, "examples/paper_workspace_calibration.py", "--help"],
        cwd=root,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "--reference-height-mm" in result.stdout
    assert "--hover-height-mm" in result.stdout
    assert "--workspace-output" in result.stdout
    assert "--measure-only" in result.stdout
    assert "--upgrade-elevated" in result.stdout
    assert "--replay" in result.stdout
    assert "--speed-mm-s" in result.stdout
    assert "--settle-tolerance-deg" in result.stdout
    assert "--settle-timeout-s" in result.stdout


def test_old_paper_hover_filename_is_safe_compatibility_entry_point() -> None:
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [sys.executable, "examples/paper_four_corner_hover_demo.py", "--help"],
        cwd=root,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "--reference-height-mm" in result.stdout
    assert "--measure-only" in result.stdout


def test_clockwise_letter_perimeter_distances() -> None:
    module = _load_example_module()
    width_m = 0.2159
    height_m = 0.2794
    points = {
        "A": np.array([0.0, 0.0, 0.0]),
        "B": np.array([width_m, 0.0, 0.0]),
        "C": np.array([width_m, height_m, 0.0]),
        "D": np.array([0.0, height_m, 0.0]),
    }

    distances = module.perimeter_distances_mm(points)

    assert distances["AB"] == pytest.approx(215.9)
    assert distances["BC"] == pytest.approx(279.4)
    assert distances["CD"] == pytest.approx(215.9)
    assert distances["DA"] == pytest.approx(279.4)
    expected_diagonal = float(np.hypot(215.9, 279.4))
    assert distances["AC"] == pytest.approx(expected_diagonal)
    assert distances["BD"] == pytest.approx(expected_diagonal)

def test_workspace_orientation_drift_is_diagnostic_only() -> None:
    module = _load_example_module()

    class Calibration:
        table_plane_rms_m = 0.00092
        affine_fit_rms_m = 0.0087
        linear_condition_number = 1.41
        model_up_scale = 0.915
        up_vs_table_normal_angle_deg = 16.02

    accepted, checks = module.evaluate_measurement_acceptance(
        Calibration(),
        up_orientation_drift_deg=16.52,
        max_table_fit_rms_mm=5.0,
        max_affine_fit_rms_mm=10.0,
        max_linear_condition_number=20.0,
        min_up_scale=0.25,
        max_up_scale=2.0,
    )

    assert accepted is True
    assert checks["measurement_accepted"] is True
    assert checks["diagnostic_up_orientation_drift_deg"] == pytest.approx(16.52)

def test_measured_demo_targets_use_exact_taught_elevated_samples() -> None:
    module = _load_example_module()
    rotation = tuple(tuple(float(value) for value in row) for row in np.eye(3))

    def sample(name, xyz, shoulder_pan):
        return module.Sample(
            name=name,
            tcp_xyz_mm=tuple(float(value) for value in xyz),
            tcp_rpy_deg=(0.0, 0.0, 0.0),
            rotation_matrix=rotation,
            joints_rad={
                "shoulder_pan": shoulder_pan,
                "shoulder_lift": 0.1,
                "elbow_flex": -0.2,
                "wrist_flex": 0.3,
                "wrist_roll": -0.4,
            },
        )

    elevated = {
        "A_UP": sample("A_UP", (10.0, 20.0, 30.0), -0.4),
        "B_UP": sample("B_UP", (40.0, 50.0, 60.0), -0.2),
        "C_UP": sample("C_UP", (70.0, 80.0, 90.0), 0.1),
        "D_UP": sample("D_UP", (100.0, 110.0, 120.0), 0.3),
        "CENTER_UP": sample("CENTER_UP", (55.0, 65.0, 75.0), 0.0),
    }

    positions, seeds = module.measured_demo_targets(elevated)

    assert positions["D_UP"] == pytest.approx(np.array([0.100, 0.110, 0.120]))
    assert positions["A_UP"] == pytest.approx(np.array([0.010, 0.020, 0.030]))
    assert positions["B_UP"] == pytest.approx(np.array([0.040, 0.050, 0.060]))
    assert positions["C_UP"] == pytest.approx(np.array([0.070, 0.080, 0.090]))
    assert positions["D_UP_RETURN"] == pytest.approx(np.array([0.100, 0.110, 0.120]))
    assert positions["CENTER_UP"] == pytest.approx(np.array([0.055, 0.065, 0.075]))
    assert seeds["B_UP"] == pytest.approx(elevated["B_UP"].joints_rad)
    assert seeds["D_UP_RETURN"] == pytest.approx(elevated["D_UP"].joints_rad)


def test_legacy_up_sample_can_seed_elevated_upgrade() -> None:
    module = _load_example_module()
    payload = {
        "samples": [
            {
                "name": "UP",
                "tcp_xyz_mm": [1.0, 2.0, 3.0],
                "tcp_rpy_deg": [4.0, 5.0, 6.0],
                "rotation_matrix": np.eye(3).tolist(),
                "joints_rad": {
                    "shoulder_pan": 0.1,
                    "shoulder_lift": 0.2,
                    "elbow_flex": 0.3,
                    "wrist_flex": 0.4,
                    "wrist_roll": 0.5,
                },
            }
        ]
    }

    d_up = module._load_d_up_for_upgrade(payload)

    assert d_up.name == "D_UP"
    assert d_up.tcp_xyz_mm == pytest.approx((1.0, 2.0, 3.0))
    assert d_up.joints_rad["wrist_roll"] == pytest.approx(0.5)


def test_hold_until_operator_release_stops_before_waiting(monkeypatch) -> None:
    module = _load_example_module()
    events = []

    class Arm:
        def stop(self):
            events.append("stop")

    monkeypatch.setattr("builtins.input", lambda _prompt: events.append("input") or "")
    module._hold_until_operator_release(Arm(), "test hold")

    assert events == ["stop", "input"]

def test_paper_config_uses_relaxed_supervised_settle_criterion() -> None:
    from types import SimpleNamespace

    module = _load_example_module()
    args = SimpleNamespace(
        port="/dev/null",
        robot_id="so101",
        calibration=None,
        settle_tolerance_deg=3.0,
        settle_timeout_s=8.0,
    )
    config = module._resolve_config(args)

    assert config.joint_position_tolerance_rad == pytest.approx(np.deg2rad(3.0))
    assert config.motion_completion_timeout_s == pytest.approx(8.0)
    assert config.cartesian_waypoint_spacing_m == pytest.approx(0.001)

