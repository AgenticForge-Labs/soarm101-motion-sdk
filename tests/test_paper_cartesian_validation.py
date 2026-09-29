from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np


def _load_example_module():
    path = Path(__file__).parents[1] / "examples" / "paper_corner_cartesian_test.py"
    spec = importlib.util.spec_from_file_location("paper_corner_cartesian_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_letter_geometry_predicts_fourth_corner() -> None:
    module = _load_example_module()
    a = np.array([0.100, -0.050, 0.020])
    b = a + np.array([0.2159, 0.0, 0.0])
    c = a + np.array([0.0, 0.2794, 0.0])

    geometry = module.derive_paper_geometry(
        a,
        b,
        c,
        width_mm=215.9,
        height_mm=279.4,
    )

    np.testing.assert_allclose(
        geometry.predicted_far_corner_m,
        np.array([0.3159, 0.2294, 0.020]),
        atol=1e-9,
    )
    assert abs(geometry.measured_width_mm - 215.9) < 1e-9
    assert abs(geometry.measured_height_mm - 279.4) < 1e-9
    assert abs(geometry.measured_corner_angle_deg - 90.0) < 1e-9
    np.testing.assert_allclose(
        geometry.normal,
        np.array([0.0, 0.0, 1.0]),
        atol=1e-9,
    )


def test_letter_geometry_orthogonalizes_noisy_corner_measurements() -> None:
    module = _load_example_module()
    a = np.array([0.0, 0.0, 0.100])
    b = np.array([0.214, 0.010, 0.101])
    c = np.array([-0.012, 0.278, 0.099])

    geometry = module.derive_paper_geometry(
        a,
        b,
        c,
        width_mm=215.9,
        height_mm=279.4,
    )

    assert abs(float(np.dot(geometry.x_axis, geometry.y_axis))) < 1e-9
    assert geometry.normal[2] > 0.0
    assert abs(float(np.linalg.norm(geometry.x_axis)) - 1.0) < 1e-9
    assert abs(float(np.linalg.norm(geometry.y_axis)) - 1.0) < 1e-9
    assert abs(float(np.linalg.norm(geometry.normal)) - 1.0) < 1e-9
