from __future__ import annotations

import importlib.util
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest


def _load_example_module():
    root = Path(__file__).resolve().parents[1]
    path = root / "examples" / "paper_four_corner_hover_demo.py"
    spec = importlib.util.spec_from_file_location("paper_four_corner_hover_demo", path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_paper_four_corner_hover_help_runs_without_hardware() -> None:
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [sys.executable, "examples/paper_four_corner_hover_demo.py", "--help"],
        cwd=root,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "--hover-height-mm" in result.stdout
    assert "--speed-mm-s" in result.stdout


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
