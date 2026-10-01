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
    assert "--startup-lift-mm" in result.stdout
    assert "--startup-height-tolerance-mm" in result.stdout
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

def test_workspace_height_targets_preserve_workspace_xy_and_set_reference_z() -> None:
    module = _load_example_module()
    rotation = tuple(tuple(float(value) for value in row) for row in np.eye(3))

    def sample(name, joints):
        return module.Sample(
            name=name,
            tcp_xyz_mm=(0.0, 0.0, 0.0),
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
    corners = {
        "A": sample("A", {**base, "shoulder_pan": -0.4, "shoulder_lift": -0.3}),
        "B": sample("B", {**base, "shoulder_pan": 0.4, "shoulder_lift": -0.2}),
        "C": sample("C", {**base, "shoulder_pan": 0.3, "shoulder_lift": 0.4}),
        "D": sample("D", {**base, "shoulder_pan": -0.3, "shoulder_lift": 0.5}),
    }
    up = sample(
        "D_UP",
        {**base, "shoulder_pan": -0.3, "shoulder_lift": 0.35, "wrist_flex": 0.1},
    )

    class Model:
        @staticmethod
        def forward(joints, tcp=None):
            del tcp
            return module.Pose(
                np.array(
                    [
                        0.20 + 0.10 * joints["shoulder_pan"],
                        0.05 + 0.10 * joints["shoulder_lift"],
                        0.02 + 0.10 * joints["wrist_flex"],
                    ],
                    dtype=float,
                ),
                np.eye(3),
            )

    class Arm:
        model = Model()
        active_tcp = None

        @staticmethod
        def get_joint_limits():
            return {name: (-3.0, 3.0) for name in base}

    class Calibration:
        reference_height_m = 0.107

        @staticmethod
        def physical_position_from_model(position):
            position = np.asarray(position, dtype=float)
            return np.array(
                [
                    2.0 * position[0] + 0.1 * position[2],
                    3.0 * position[1] - 0.2 * position[2],
                    4.0 * position[2],
                ]
            )

        @staticmethod
        def model_position_from_physical(x, y, z):
            model_z = z / 4.0
            return np.array(
                [
                    (x - 0.1 * model_z) / 2.0,
                    (y + 0.2 * model_z) / 3.0,
                    model_z,
                ]
            )

    positions, seeds, baseline_z_mm = module.workspace_height_demo_targets(
        Calibration(),
        Arm(),
        corners=corners,
        up_sample=up,
    )

    baseline, _ = module._inferred_reachable_demo_targets(
        Arm(),
        corners=corners,
        up_sample=up,
    )
    for name, target in positions.items():
        before = Calibration.physical_position_from_model(baseline[name])
        after = Calibration.physical_position_from_model(target)
        assert after[:2] == pytest.approx(before[:2])
        assert after[2] == pytest.approx(0.107)
        assert baseline_z_mm[name] == pytest.approx(before[2] * 1000.0)
        assert name in seeds


def test_workspace_z_offset_target_changes_only_workspace_z() -> None:
    module = _load_example_module()

    class Calibration:
        @staticmethod
        def physical_position_from_model(position):
            position = np.asarray(position, dtype=float)
            return np.array([position[0] + position[2], position[1], 2.0 * position[2]])

        @staticmethod
        def model_position_from_physical(x, y, z):
            model_z = z / 2.0
            return np.array([x - model_z, y, model_z])

    model = np.array([0.10, 0.20, 0.03])
    target, start_physical, target_physical = module.workspace_z_offset_target(
        Calibration(),
        model,
        delta_z_m=0.010,
    )

    assert target_physical[:2] == pytest.approx(start_physical[:2])
    assert target_physical[2] - start_physical[2] == pytest.approx(0.010)
    assert Calibration.physical_position_from_model(target) == pytest.approx(target_physical)


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
    assert config.command_frequency_hz == pytest.approx(20.0)
    assert config.cartesian_waypoint_spacing_m == pytest.approx(0.001)



def test_startup_lift_preflight_uses_calibrated_workspace_when_model_z_is_negative() -> None:
    from types import SimpleNamespace

    module = _load_example_module()
    joint_names = (
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
    )
    start_joints = {name: 0.0 for name in joint_names}
    start_joints["wrist_flex"] = -0.020

    class Calibration:
        @staticmethod
        def physical_position_from_model(position):
            position = np.asarray(position, dtype=float)
            return np.array([position[0], position[1], position[2] + 0.020])

        @staticmethod
        def model_position_from_physical(x, y, z):
            return np.array([x, y, z - 0.020])

    class Model:
        @staticmethod
        def forward(joints, tcp=None):
            del tcp
            return module.Pose(
                np.array([0.0, 0.0, float(joints["wrist_flex"])]),
                np.eye(3),
            )

    class IK:
        @staticmethod
        def solve(target, *, seed, tcp, options):
            del seed, tcp, options
            joints = {name: 0.0 for name in joint_names}
            joints["wrist_flex"] = float(target.position[2])
            return SimpleNamespace(
                success=True,
                joints=joints,
                position_error_m=0.0,
                message="ok",
            )

    class Arm:
        config = SimpleNamespace(
            cartesian_position_tolerance_m=0.0005,
            max_ik_waypoint_jump_radians=0.50,
        )
        active_tcp = None
        model = Model()
        ik = IK()

        @staticmethod
        def get_joint_limits():
            return {name: (-3.0, 3.0) for name in joint_names}

    start_pose = module.Pose(np.array([0.0, 0.0, -0.020]), np.eye(3))
    result = module.preflight_calibrated_workspace_z_lift(
        Arm(),
        Calibration(),
        start_pose=start_pose,
        start_joints=start_joints,
        target_workspace_z_m=0.010,
    )

    # The final model-frame Z remains below the generic zero floor, but calibrated
    # workspace Z rises cleanly from 0 to 10 mm and must be accepted.
    assert result["target_model_xyz_mm"][2] == pytest.approx(-10.0)
    assert result["start_workspace_z_mm"] == pytest.approx(0.0)
    assert result["target_workspace_z_mm"] == pytest.approx(10.0)
    assert result["max_workspace_xy_drift_mm"] == pytest.approx(0.0)
    assert result["generic_model_workspace_floor_bypassed"] is True


def test_preflighted_startup_lift_executes_with_generic_workspace_check_off() -> None:
    from types import SimpleNamespace

    module = _load_example_module()
    calls = []

    class Arm:
        def move_linear(self, target, **kwargs):
            calls.append((target, kwargs))
            return SimpleNamespace(accepted=True, completed=True)

    result = module.execute_preflighted_workspace_z_lift(
        Arm(),
        target_model_position_m=np.array([0.1, 0.2, -0.01]),
        rotation=np.eye(3),
        speed_mm_s=20.0,
        acceleration_mm_s2=100.0,
    )

    assert result.completed is True
    assert len(calls) == 1
    target, kwargs = calls[0]
    assert target.position == pytest.approx(np.array([0.1, 0.2, -0.01]))
    assert kwargs["workspace_check"] == "off"
    assert kwargs["orientation_mode"] == "position_only"
    assert kwargs["speed"] == pytest.approx(0.020)
    assert kwargs["acceleration"] == pytest.approx(0.100)


def test_startup_transport_height_reaches_reference_before_lateral_travel() -> None:
    module = _load_example_module()

    assert module.startup_transport_height_m(
        current_workspace_z_m=0.0004,
        reference_height_m=0.107,
        minimum_lift_m=0.010,
    ) == pytest.approx(0.107)

    assert module.startup_transport_height_m(
        current_workspace_z_m=0.120,
        reference_height_m=0.107,
        minimum_lift_m=0.010,
    ) == pytest.approx(0.130)
