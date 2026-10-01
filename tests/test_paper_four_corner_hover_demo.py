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

def test_constant_height_demo_targets_use_workspace_geometry_and_seed_only_lift() -> None:
    module = _load_example_module()
    rotation = tuple(tuple(float(value) for value in row) for row in np.eye(3))
    width = 0.2159
    height = 0.2794
    reference_height = 0.107
    origin = np.array([0.12, -0.03, -0.04])
    linear = np.array(
        [
            [0.80, 0.05, 0.20],
            [0.10, 0.95, -0.10],
            [0.15, -0.12, 0.90],
        ]
    )

    def model(point):
        return origin + linear @ np.asarray(point, dtype=float)

    def sample(name, position, joints):
        return module.Sample(
            name=name,
            tcp_xyz_mm=tuple(float(value * 1000.0) for value in position),
            tcp_rpy_deg=(0.0, 0.0, 0.0),
            rotation_matrix=rotation,
            joints_rad=dict(joints),
        )

    base = {
        "shoulder_pan": 0.0,
        "shoulder_lift": 0.0,
        "elbow_flex": 0.0,
        "wrist_flex": 0.0,
        "wrist_roll": 0.0,
    }
    physical_corners = {
        "A": (0.0, 0.0, 0.0),
        "B": (width, 0.0, 0.0),
        "C": (width, height, 0.0),
        "D": (0.0, height, 0.0),
    }
    corner_positions = {
        name: model(point) for name, point in physical_corners.items()
    }
    corners = {
        "A": sample(
            "A",
            corner_positions["A"],
            {**base, "shoulder_pan": -0.4, "shoulder_lift": -0.3},
        ),
        "B": sample(
            "B",
            corner_positions["B"],
            {**base, "shoulder_pan": 0.4, "shoulder_lift": -0.2},
        ),
        "C": sample(
            "C",
            corner_positions["C"],
            {**base, "shoulder_pan": 0.3, "shoulder_lift": 0.4},
        ),
        "D": sample(
            "D",
            corner_positions["D"],
            {**base, "shoulder_pan": -0.3, "shoulder_lift": 0.5},
        ),
    }
    up_position = model((0.0, height, reference_height))
    up = sample(
        "UP",
        up_position,
        {**base, "shoulder_pan": -0.3, "shoulder_lift": 0.35, "wrist_flex": 0.1},
    )
    calibration = module.fit_paper_workspace(
        corner_positions,
        up_position,
        robot_id="so101",
        arm_calibration_id="sha256:motor",
        width_m=width,
        height_m=height,
        reference_height_m=reference_height,
    )

    positions, seeds = module.constant_height_demo_targets(
        calibration,
        corners=corners,
        up_sample=up,
    )

    expected_xy = {
        "D_UP": (0.0, height),
        "A_UP": (0.0, 0.0),
        "B_UP": (width, 0.0),
        "C_UP": (width, height),
        "D_UP_RETURN": (0.0, height),
        "CENTER_UP": (width / 2.0, height / 2.0),
    }
    for name, position in positions.items():
        physical = calibration.physical_position_from_model(position)
        assert physical[:2] == pytest.approx(expected_xy[name], abs=1e-9)
        assert physical[2] == pytest.approx(reference_height, abs=1e-9)

    lift = {
        name: up.joints_rad[name] - corners["D"].joints_rad[name]
        for name in base
    }
    expected_b_seed = {
        name: corners["B"].joints_rad[name] + lift[name]
        for name in base
    }
    assert seeds["D_UP"] == pytest.approx(up.joints_rad)
    assert seeds["B_UP"] == pytest.approx(expected_b_seed)

    # Constant physical height does not imply constant model/base-frame Z.
    model_z = [float(position[2]) for position in positions.values()]
    assert max(model_z) - min(model_z) > 0.01


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

