import math

import pytest

from soarm101_motion.exceptions import InvalidJointError, SafetyViolationError
from soarm101_motion.safety import validate_joint_targets


def test_valid_target() -> None:
    assert validate_joint_targets({"shoulder_pan": 0.2}) == {"shoulder_pan": 0.2}


def test_unknown_joint() -> None:
    with pytest.raises(InvalidJointError):
        validate_joint_targets({"camera": 0.0})


def test_nonfinite_target() -> None:
    with pytest.raises(SafetyViolationError):
        validate_joint_targets({"shoulder_pan": math.inf})


def test_out_of_range_target() -> None:
    with pytest.raises(SafetyViolationError):
        validate_joint_targets({"gripper": 2.0})
