from __future__ import annotations

import math

import pytest

from soarm101_motion import SOARM101
from soarm101_motion.control import jog_linear_cli_units, relative_target_pose
from soarm101_motion.exceptions import InvalidCommandError
from soarm101_motion.types import Pose


def test_position_only_rotation_jog_is_rejected() -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        with pytest.raises(InvalidCommandError, match="position_only ignores orientation"):
            jog_linear_cli_units(
                arm,
                frame="world",
                rotation_rpy_deg=(0.0, 0.0, 2.0),
                orientation_mode="position_only",
            )


def test_empty_jog_is_rejected() -> None:
    with SOARM101.simulated() as arm:
        arm.enable()
        with pytest.raises(InvalidCommandError, match="must include a translation or rotation"):
            jog_linear_cli_units(arm, frame="tool")


def test_nonfinite_jog_value_is_rejected() -> None:
    with pytest.raises(InvalidCommandError, match="must be finite"):
        relative_target_pose(
            Pose.identity(),
            translation_m=(math.nan, 0.0, 0.0),
            frame="world",
        )
