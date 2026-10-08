from __future__ import annotations

import numpy as np
import pytest

from soarm101_motion.exceptions import SafetyViolationError
from soarm101_motion.safety import validate_workspace_path_from_measured_start


class _MeasuredStartModel:
    def link_points(self, joints, *, tcp=None):
        del tcp
        clearance = float(joints["clearance"])
        z = -0.01 if bool(joints.get("floor_violation", False)) else 0.20
        return {
            "shoulder_pan": np.array([0.20, 0.00, 0.20]),
            "shoulder_lift": np.array([0.30, 0.00, 0.20]),
            "elbow_flex": np.array([0.30, clearance, z]),
            "wrist_flex": np.array([0.20, clearance, 0.20]),
            "wrist_roll": np.array([0.20, clearance, 0.30]),
            "tcp": np.array([0.25, clearance, 0.35]),
        }


def test_measured_start_self_clearance_can_improve_without_fully_exiting() -> None:
    model = _MeasuredStartModel()
    validate_workspace_path_from_measured_start(
        model,
        (
            {"clearance": 0.021},
            {"clearance": 0.022},
            {"clearance": 0.024},
        ),
        minimum_self_clearance_m=0.025,
    )


def test_measured_start_self_clearance_cannot_move_deeper() -> None:
    model = _MeasuredStartModel()
    with pytest.raises(SafetyViolationError, match="moves deeper"):
        validate_workspace_path_from_measured_start(
            model,
            (
                {"clearance": 0.021},
                {"clearance": 0.019},
            ),
            minimum_self_clearance_m=0.025,
        )


def test_measured_start_resumes_strict_clearance_after_exiting() -> None:
    model = _MeasuredStartModel()
    with pytest.raises(SafetyViolationError, match="coarse self-clearance"):
        validate_workspace_path_from_measured_start(
            model,
            (
                {"clearance": 0.021},
                {"clearance": 0.026},
                {"clearance": 0.023},
            ),
            minimum_self_clearance_m=0.025,
        )


def test_measured_start_does_not_relax_floor_guard() -> None:
    model = _MeasuredStartModel()
    with pytest.raises(SafetyViolationError, match="below 0.000 m"):
        validate_workspace_path_from_measured_start(
            model,
            (
                {"clearance": 0.021},
                {"clearance": 0.022, "floor_violation": True},
            ),
            minimum_self_clearance_m=0.025,
        )
