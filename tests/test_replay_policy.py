import math

import pytest

from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.replay_policy import ReplayPolicy


def policy_document():
    return {
        "schema_version": 1, "robot_id": "arm", "calibration_id": "cal-1",
        "allowed_assets": {"soarm101/recording/raw/wave": "sha256:" + "a" * 64},
        "max_speed_scale": 0.5, "max_steps": 20, "max_duration_s": 30,
        "max_start_error_rad": 0.05,
        "max_start_gripper_error": 0.05,
    }


def test_policy_pins_identity_and_revision():
    policy = ReplayPolicy.from_document(policy_document())
    policy.require_asset("soarm101/recording/raw/wave", "sha256:" + "a" * 64)
    with pytest.raises(PermissionError, match="revision"):
        policy.require_asset("soarm101/recording/raw/wave", "sha256:" + "b" * 64)
    with pytest.raises(PermissionError, match="not allowed"):
        policy.require_asset("soarm101/recording/raw/other", "sha256:" + "a" * 64)


@pytest.mark.parametrize("key,value", [
    ("max_speed_scale", 1.01), ("max_speed_scale", True), ("max_steps", 0),
    ("max_duration_s", math.inf), ("max_start_error_rad", 0.11),
    ("calibration_id", ""), ("allowed_assets", {"../../arbitrary": "sha256:" + "a" * 64}),
])
def test_invalid_policy_is_rejected(key, value):
    data = policy_document()
    data[key] = value
    with pytest.raises(ValueError):
        ReplayPolicy.from_document(data)


def test_start_state_is_complete_finite_and_within_tolerance():
    policy = ReplayPolicy.from_document(policy_document())
    expected = dict.fromkeys(ARM_JOINTS, 0.0)
    policy.check_start(expected, expected)
    actual = {**expected, ARM_JOINTS[0]: 0.06}
    with pytest.raises(PermissionError, match="tolerance"):
        policy.check_start(actual, expected)
    with pytest.raises(ValueError, match="exactly"):
        policy.check_start({}, expected)
    with pytest.raises(ValueError, match="finite"):
        policy.check_start({**expected, ARM_JOINTS[0]: math.nan}, expected)


def test_direct_policy_construction_cannot_bypass_bounds():
    with pytest.raises(ValueError, match="bound"):
        ReplayPolicy("arm", "cal-1", (("soarm101/pose/demo", "sha256:" + "a" * 64),),
                     10.0, 1, 10.0, 0.05, 0.05)
