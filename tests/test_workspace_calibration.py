from __future__ import annotations

import numpy as np
import pytest

from soarm101_motion.workspace import (
    WorkspaceCalibrationStore,
    fit_paper_workspace,
)


def _apply_affine(
    origin: np.ndarray,
    linear: np.ndarray,
    point: tuple[float, float, float],
) -> np.ndarray:
    return origin + linear @ np.asarray(point, dtype=float)


def test_paper_workspace_recovers_known_local_affine_mapping(tmp_path) -> None:
    width = 0.2159
    height = 0.2794
    up_height = 0.050
    origin = np.array([0.120, -0.030, -0.040])
    linear = np.array(
        [
            [0.80, 0.05, 0.20],
            [0.10, 0.95, -0.10],
            [0.02, -0.04, 0.90],
        ]
    )
    physical = {
        "A": (0.0, 0.0, 0.0),
        "B": (width, 0.0, 0.0),
        "C": (width, height, 0.0),
        "D": (0.0, height, 0.0),
    }
    corners = {
        name: _apply_affine(origin, linear, point)
        for name, point in physical.items()
    }
    up = _apply_affine(origin, linear, (0.0, height, up_height))

    calibration = fit_paper_workspace(
        corners,
        up,
        robot_id="so101",
        arm_calibration_id="sha256:motor",
        width_m=width,
        height_m=height,
        reference_height_m=up_height,
    )

    assert np.asarray(calibration.model_origin_m) == pytest.approx(origin, abs=1e-10)
    assert np.asarray(calibration.physical_to_model_linear) == pytest.approx(
        linear, abs=1e-10
    )
    assert calibration.affine_fit_rms_m < 1e-10
    assert calibration.table_plane_rms_m < 1e-10
    expected = _apply_affine(origin, linear, (0.1, 0.2, 0.03))
    assert calibration.model_position_from_physical(0.1, 0.2, 0.03) == pytest.approx(
        expected
    )
    assert calibration.motion_validation_status == "unvalidated"

    path = tmp_path / "workspace.json"
    store = WorkspaceCalibrationStore("so101", path=path)
    store.save(calibration)
    loaded = store.load()
    assert loaded.workspace_id == calibration.workspace_id
    assert loaded.arm_calibration_id == "sha256:motor"
    assert np.asarray(loaded.physical_to_model_linear) == pytest.approx(linear)


def test_paper_workspace_detects_up_direction_inconsistent_with_table_normal() -> None:
    width = 0.2159
    height = 0.2794
    up_height = 0.050
    corners = {
        "A": np.array([0.0, 0.0, 0.0]),
        "B": np.array([width, 0.0, 0.0]),
        "C": np.array([width, height, 0.0]),
        "D": np.array([0.0, height, 0.0]),
    }
    # The operator says this is physically UP, but the model maps it almost
    # entirely along the table. That is exactly the condition that must block
    # Cartesian hover activation.
    up = np.array([0.050, height, 0.002])

    calibration = fit_paper_workspace(
        corners,
        up,
        robot_id="so101",
        arm_calibration_id="sha256:motor",
        width_m=width,
        height_m=height,
        reference_height_m=up_height,
    )

    assert calibration.up_vs_table_normal_angle_deg > 80.0
    assert calibration.up_reference_tangent_m > 0.045
    assert calibration.up_reference_plane_height_m < 0.005
