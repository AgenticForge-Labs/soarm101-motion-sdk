from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest

from soarm101_motion.workspace import fit_paper_workspace

_EXAMPLE_PATH = Path(__file__).parents[1] / "examples" / "paper_workspace_calibration.py"
_SPEC = importlib.util.spec_from_file_location("paper_workspace_calibration_example", _EXAMPLE_PATH)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
Sample = _MODULE.Sample
constant_height_demo_targets = _MODULE.constant_height_demo_targets


def _sample(name: str, position: np.ndarray, joints: dict[str, float]) -> Sample:
    return Sample(
        name=name,
        tcp_xyz_mm=tuple(float(value * 1000.0) for value in position),
        tcp_rpy_deg=(0.0, 0.0, 0.0),
        rotation_matrix=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        joints_rad=joints,
    )


def test_demo_targets_share_the_taught_physical_height() -> None:
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

    def model(point: tuple[float, float, float]) -> np.ndarray:
        return origin + linear @ np.asarray(point, dtype=float)

    physical_corners = {
        "A": (0.0, 0.0, 0.0),
        "B": (width, 0.0, 0.0),
        "C": (width, height, 0.0),
        "D": (0.0, height, 0.0),
    }
    corner_positions = {name: model(point) for name, point in physical_corners.items()}
    up_position = model((0.0, height, reference_height))
    calibration = fit_paper_workspace(
        corner_positions,
        up_position,
        robot_id="so101",
        arm_calibration_id="sha256:motor",
        width_m=width,
        height_m=height,
        reference_height_m=reference_height,
    )

    zero_joints = {
        "shoulder_pan": 0.0,
        "shoulder_lift": 0.0,
        "elbow_flex": 0.0,
        "wrist_flex": 0.0,
        "wrist_roll": 0.0,
    }
    corners = {
        name: _sample(name, position, dict(zero_joints))
        for name, position in corner_positions.items()
    }
    up = _sample(
        "UP",
        up_position,
        {**zero_joints, "shoulder_lift": 0.1, "elbow_flex": -0.1},
    )

    positions, seeds = constant_height_demo_targets(
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

    # A constant physical height need not be a constant model/base-frame Z when
    # the measured workspace transform is skewed.
    model_z = [float(position[2]) for position in positions.values()]
    assert max(model_z) - min(model_z) > 0.01
    assert set(seeds) == set(positions)
