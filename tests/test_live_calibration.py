import pytest

from soarm101_motion.calibration_live import (
    PROVISIONAL_MINIMUM_TRAVEL_TICKS,
    EncoderSweep,
    calibration_from_sweeps,
    display_travel_targets,
    sweep_progress_snapshot,
)
from soarm101_motion.constants import ALL_MOTORS, ENCODER_RESOLUTION, HALF_TURN
from soarm101_motion.exceptions import CalibrationError


def sweep(values: list[int]) -> EncoderSweep:
    result = EncoderSweep.start(values[0])
    for value in values[1:]:
        result.update(value)
    return result


def test_encoder_sweep_unwraps_across_zero_seam() -> None:
    result = sweep([3500, 3900, 100, 500, 700])
    assert result.minimum == 3500
    assert result.maximum == 4796
    assert result.travel_ticks == 1296
    assert result.seam_crossings == 1


def test_live_calibration_maps_mechanical_midpoint_to_half_turn() -> None:
    # Use a seam-crossing sweep for every motor to exercise the difficult case.
    sweeps = {
        name: sweep([2800, 3500, 100, 700, 1200, 700, 100, 3500, 2800])
        for name in ALL_MOTORS
    }
    calibration = calibration_from_sweeps(sweeps)

    for motor in calibration.motors.values():
        assert motor.range_min + motor.range_max == pytest.approx(2 * HALF_TURN, abs=1)
        assert motor.range_min < HALF_TURN < motor.range_max
        assert abs(motor.homing_offset) < ENCODER_RESOLUTION // 2
        assert motor.raw_to_radians(HALF_TURN) == pytest.approx(0.0, abs=0.002)


def test_live_calibration_rejects_motor_not_swept_to_stops() -> None:
    sweeps = {name: sweep([2000, 2050, 2100]) for name in ALL_MOTORS}
    with pytest.raises(CalibrationError, match="sweep it repeatedly"):
        calibration_from_sweeps(sweeps)


def test_live_calibration_rejects_more_than_one_encoder_turn() -> None:
    sweeps = {
        name: sweep([100, 1000, 2000, 3000, 3900, 800, 1800, 2800, 3800, 700])
        for name in ALL_MOTORS
    }
    with pytest.raises(CalibrationError, match="one full encoder turn or more"):
        calibration_from_sweeps(sweeps)


def test_default_threshold_requires_half_turn_for_pose_joints() -> None:
    assert PROVISIONAL_MINIMUM_TRAVEL_TICKS["shoulder_pan"] == ENCODER_RESOLUTION // 2
    assert PROVISIONAL_MINIMUM_TRAVEL_TICKS["shoulder_lift"] == ENCODER_RESOLUTION // 2
    assert PROVISIONAL_MINIMUM_TRAVEL_TICKS["elbow_flex"] == ENCODER_RESOLUTION // 2
    assert PROVISIONAL_MINIMUM_TRAVEL_TICKS["wrist_flex"] == ENCODER_RESOLUTION // 2
    assert PROVISIONAL_MINIMUM_TRAVEL_TICKS["wrist_roll"] == ENCODER_RESOLUTION // 2
    assert PROVISIONAL_MINIMUM_TRAVEL_TICKS["so101_gripper"] == 900


def test_live_calibration_rejects_pose_joint_below_half_turn() -> None:
    complete = sweep([1000, 2000, 3000, 3100])
    short = sweep([1000, 1800, 2800])
    gripper = sweep([2000, 2300])
    sweeps = {name: complete for name in ALL_MOTORS}
    sweeps = dict(sweeps)
    sweeps["shoulder_pan"] = short
    sweeps["so101_gripper"] = gripper

    with pytest.raises(CalibrationError, match=r"shoulder_pan moved only 1800.*2048"):
        calibration_from_sweeps(sweeps)


def test_gripper_keeps_separate_provisional_threshold() -> None:
    complete = sweep([1000, 2000, 3000, 3100, 2000, 1000])
    gripper = sweep([1000, 1600, 2100, 1600, 1000])
    sweeps = {name: complete for name in ALL_MOTORS}
    sweeps = dict(sweeps)
    sweeps["so101_gripper"] = gripper

    calibration = calibration_from_sweeps(sweeps)
    assert calibration.motors["so101_gripper"].range_max > calibration.motors["so101_gripper"].range_min


def test_sweep_progress_reports_fraction_ticks_and_pass_state() -> None:
    sweeps = {name: sweep([1000, 1500]) for name in ALL_MOTORS}
    progress = sweep_progress_snapshot(sweeps)

    assert progress["shoulder_pan"]["travel_ticks"] == 500
    assert progress["shoulder_pan"]["required_ticks"] == 2048
    assert progress["shoulder_pan"]["fraction"] == pytest.approx(500 / 2200)
    assert progress["shoulder_pan"]["passed"] is False

    sweeps["shoulder_pan"] = sweep([1000, 2000, 3000, 3100])
    progress = sweep_progress_snapshot(sweeps)
    assert progress["shoulder_pan"]["fraction"] == pytest.approx(2100 / 2200)
    assert progress["shoulder_pan"]["passed"] is False
    assert progress["shoulder_pan"]["sweep_number"] == 1

    sweeps["shoulder_pan"] = sweep([1000, 2000, 3000, 3100, 2000, 1000])
    progress = sweep_progress_snapshot(sweeps)
    assert progress["shoulder_pan"]["sweep_number"] == 2
    assert progress["shoulder_pan"]["passed"] is True


def test_partial_leg_remains_visible_after_reversal_without_counting_as_complete() -> None:
    first = sweep([1000, 1500, 1800, 1750, 1400, 1100])
    progress = sweep_progress_snapshot({"shoulder_pan": first})["shoulder_pan"]
    assert progress["active_leg_ticks"] < progress["best_leg_ticks"]
    assert progress["best_leg_ticks"] == 800
    assert progress["fraction"] == pytest.approx(800 / 2200)
    assert progress["traversals_completed"] == 0
    assert progress["passed"] is False


def test_wrist_roll_gauge_reaches_full_near_measured_mechanical_range() -> None:
    sweeps = {name: sweep([1000, 1500]) for name in ALL_MOTORS}
    sweeps["wrist_roll"] = sweep([100, 2148])
    progress = sweep_progress_snapshot(sweeps)["wrist_roll"]
    assert progress["passed"] is False
    assert progress["display_ticks"] == 3600
    assert progress["fraction"] == pytest.approx(2048 / 3600)

    sweeps["wrist_roll"] = sweep([100, 1000, 2000, 3000, 3700])
    assert sweep_progress_snapshot(sweeps)["wrist_roll"]["fraction"] == 1.0
    assert sweep_progress_snapshot(sweeps)["wrist_roll"]["passed"] is False

    sweeps["wrist_roll"] = sweep([100, 1000, 2000, 3000, 3700, 3000, 2000, 1000, 100])
    assert sweep_progress_snapshot(sweeps)["wrist_roll"]["passed"] is True


def test_two_full_traversals_are_required_to_save() -> None:
    once = sweep([1000, 2000, 3100])
    sweeps = {name: once for name in ALL_MOTORS}
    with pytest.raises(CalibrationError, match="1/2 full end-to-end traversals"):
        calibration_from_sweeps(sweeps)

    twice = sweep([1000, 2000, 3100, 2000, 1000])
    gripper = sweep([1000, 1600, 2200, 1600, 1000])
    sweeps = {name: twice for name in ALL_MOTORS}
    sweeps["so101_gripper"] = gripper
    assert calibration_from_sweeps(sweeps).motors["so101_gripper"].range_max > 0


def test_small_reversals_do_not_count_as_full_traversals() -> None:
    joint = sweep([1000, 2000, 3000, 3100, 3090, 3100, 3050])
    assert joint.completed_traversals(2048) == 1
    assert joint.finalized_traversals(2048) == 1


def test_gauge_uses_selected_arms_prior_gripper_range(tmp_path, monkeypatch) -> None:
    twice = sweep([1000, 2000, 3200, 2000, 1000])
    gripper = sweep([1000, 1600, 2188, 1600, 1000])
    sweeps = {name: twice for name in ALL_MOTORS}
    sweeps["so101_gripper"] = gripper
    path = calibration_from_sweeps(sweeps).save(tmp_path / "leader.json")
    monkeypatch.setattr(
        "soarm101_motion.calibration_live.default_calibration_path",
        lambda _robot_id: path,
    )
    targets = display_travel_targets("leader")
    assert targets["so101_gripper"] == round(0.95 * 1188)
    progress = sweep_progress_snapshot(
        {"so101_gripper": sweep([1000, 1600, 2188])},
        display_travel_ticks=targets,
    )
    assert progress["so101_gripper"]["fraction"] == 1.0
