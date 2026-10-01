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
    assert "no autonomous cartesian arm motion" in result.stdout.lower()


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
    assert "no autonomous cartesian arm motion" in result.stdout.lower()


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

def test_workspace_orientation_drift_is_warning_not_rejection() -> None:
    module = _load_example_module()

    class Calibration:
        table_plane_rms_m = 0.00092
        affine_fit_rms_m = 0.0087
        linear_condition_number = 1.41
        model_up_scale = 0.915
        up_vs_table_normal_angle_deg = 16.02

    accepted, checks, warnings = module.evaluate_measurement_acceptance(
        Calibration(),
        up_orientation_drift_deg=16.52,
        max_table_fit_rms_mm=5.0,
        max_affine_fit_rms_mm=10.0,
        max_linear_condition_number=20.0,
        up_orientation_drift_warning_deg=10.0,
        min_up_scale=0.25,
        max_up_scale=2.0,
    )

    assert accepted is True
    assert checks["measurement_accepted"] is True
    assert checks["up_probe_orientation_within_warning_threshold"] is False
    assert warnings
    assert "16.52 deg" in warnings[0]

