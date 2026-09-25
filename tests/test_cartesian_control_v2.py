from __future__ import annotations

import numpy as np
import pytest

from soarm101_motion import SOARM101
from soarm101_motion.control import jog_linear_cli_units


@pytest.mark.parametrize(
    ("translation_mm", "expected_mm"),
    [
        ((2.0, 0.0, 0.0), np.array([2.0, 0.0, 0.0])),
        ((-2.0, 0.0, 0.0), np.array([-2.0, 0.0, 0.0])),
        ((0.0, 2.0, 0.0), np.array([0.0, 2.0, 0.0])),
        ((0.0, -2.0, 0.0), np.array([0.0, -2.0, 0.0])),
        ((0.0, 0.0, 2.0), np.array([0.0, 0.0, 2.0])),
        ((0.0, 0.0, -2.0), np.array([0.0, 0.0, -2.0])),
    ],
)
def test_small_world_jogs_move_along_requested_axis(
    translation_mm: tuple[float, float, float],
    expected_mm: np.ndarray,
) -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        before = arm.get_position().position.copy()
        jog_linear_cli_units(
            arm,
            frame="world",
            translation_mm=translation_mm,
            orientation_mode="compatible",
            speed_mm_s=10.0,
            acceleration_mm_s2=40.0,
        )
        observed_mm = (arm.get_position().position - before) * 1000.0

    assert observed_mm == pytest.approx(expected_mm, abs=0.5)


def test_x_and_y_jogs_produce_distinct_joint_solutions() -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        start = arm.get_joint_positions().positions
        current = arm.get_position()

        x_target = current.position + np.array([-0.002, 0.0, 0.0])
        y_target = current.position + np.array([0.0, -0.002, 0.0])

        from soarm101_motion.types import Pose

        x_solution = arm.solve_ik(
            Pose(x_target, current.rotation),
            seed=start,
            orientation_mode="compatible",
        )
        y_solution = arm.solve_ik(
            Pose(y_target, current.rotation),
            seed=start,
            orientation_mode="compatible",
        )

    assert x_solution.success
    assert y_solution.success
    joint_delta = max(
        abs(x_solution.joints[name] - y_solution.joints[name])
        for name in x_solution.joints
    )
    assert joint_delta > 0.005
