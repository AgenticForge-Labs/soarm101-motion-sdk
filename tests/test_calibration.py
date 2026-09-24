import json

import pytest

from soarm101_motion.calibration import MotorCalibration, SO101Calibration
from soarm101_motion.constants import ALL_MOTORS, HALF_TURN, MOTOR_IDS, STOCK_GRIPPER


def sample_calibration() -> SO101Calibration:
    return SO101Calibration(
        motors={
            name: MotorCalibration(MOTOR_IDS[name], 0, 0, 100, 3995)
            for name in ALL_MOTORS
        }
    )


def test_failed_calibration_replace_keeps_previous_file(tmp_path, monkeypatch) -> None:
    import soarm101_motion.calibration as storage

    path = tmp_path / "so101.json"
    path.write_text("old calibration\n", encoding="utf-8")

    def fail_replace(_source, _target):
        raise OSError("simulated replace failure")

    monkeypatch.setattr(storage.os, "replace", fail_replace)
    with pytest.raises(OSError, match="simulated replace failure"):
        sample_calibration().save(path)
    assert path.read_text(encoding="utf-8") == "old calibration\n"
    assert sorted(tmp_path.iterdir()) == [path]


def test_joint_calibration_round_trip() -> None:
    calibration = MotorCalibration(1, 0, 0, 100, 3995)
    for radians in (-1.0, 0.0, 1.0):
        recovered = calibration.raw_to_radians(calibration.radians_to_raw(radians))
        assert recovered == pytest.approx(radians, abs=0.002)


def test_joint_zero_is_calibrated_midpoint_for_asymmetric_range() -> None:
    calibration = MotorCalibration(1, 0, 123, 400, 3600)
    assert calibration.center_raw == pytest.approx(2000.0)
    assert calibration.raw_to_radians(2000) == pytest.approx(0.0)
    assert calibration.radians_to_raw(0.0) == 2000
    assert calibration.raw_to_radians(HALF_TURN) > 0.0


def test_asymmetric_joint_range_does_not_need_to_include_half_turn() -> None:
    calibration = SO101Calibration(
        motors={
            "shoulder_pan": MotorCalibration(MOTOR_IDS["shoulder_pan"], 0, 0, 100, 1000)
        }
    )
    calibration.validate(require_all=False)
    motor = calibration.motors["shoulder_pan"]
    assert motor.center_raw == pytest.approx(550.0)
    assert motor.raw_to_radians(550) == pytest.approx(0.0)


def test_asymmetric_joint_limits_are_centered_on_zero() -> None:
    calibration = MotorCalibration(1, 0, 0, 900, 3300)
    lower, upper = calibration.radians_limits
    assert lower == pytest.approx(-upper)
    assert calibration.center_raw == pytest.approx(2100.0)


def test_native_symmetric_calibration_still_uses_half_turn_zero() -> None:
    calibration = MotorCalibration(1, 0, 0, 900, 3194)
    assert calibration.center_raw == pytest.approx(HALF_TURN)
    assert calibration.raw_to_radians(HALF_TURN) == pytest.approx(0.0)
    assert calibration.radians_to_raw(0.0) == HALF_TURN


def test_gripper_range_need_not_include_half_turn() -> None:
    calibration = SO101Calibration(
        motors={STOCK_GRIPPER: MotorCalibration(MOTOR_IDS[STOCK_GRIPPER], 0, 0, 100, 1100)}
    )
    calibration.validate(require_all=False)


def test_factory_range_is_reported_uncalibrated_for_every_motor() -> None:
    calibration = SO101Calibration(
        motors={name: MotorCalibration(MOTOR_IDS[name], 0, 0, 0, 4095) for name in ALL_MOTORS}
    )
    assert set(calibration.uncalibrated_motors) == set(ALL_MOTORS)


def test_gripper_calibration_round_trip_and_inversion() -> None:
    normal = MotorCalibration(6, 0, 0, 100, 1100)
    inverted = MotorCalibration(6, 1, 0, 100, 1100)
    assert normal.raw_to_normalized(100) == 0.0
    assert normal.raw_to_normalized(1100) == 1.0
    assert inverted.raw_to_normalized(100) == 1.0
    assert inverted.normalized_to_raw(0.25) == 850


def test_imported_lerobot_asymmetric_range_uses_recorded_midpoint_zero() -> None:
    calibration = SO101Calibration.from_mapping(
        {
            "shoulder_pan": {
                "id": MOTOR_IDS["shoulder_pan"],
                "drive_mode": 0,
                "homing_offset": 53,
                "range_min": 900,
                "range_max": 3300,
            }
        }
    )
    motor = calibration.motors["shoulder_pan"]
    assert calibration.source == "lerobot"
    assert motor.center_raw == pytest.approx(2100.0)
    assert motor.raw_to_radians(2100) == pytest.approx(0.0)
    assert motor.radians_to_raw(0.0) == 2100
    assert motor.raw_to_radians(HALF_TURN) < 0.0


def test_load_and_export_lerobot_format(tmp_path) -> None:
    calibration = sample_calibration()
    own_path = tmp_path / "calibration.json"
    lerobot_path = tmp_path / "lerobot.json"
    calibration.save(own_path)
    calibration.save_lerobot(lerobot_path)

    assert SO101Calibration.load(own_path).motors.keys() == calibration.motors.keys()
    loaded = SO101Calibration.load(lerobot_path)
    assert STOCK_GRIPPER in loaded.motors
    assert loaded.motors["shoulder_pan"].raw_to_radians(2047.5) == pytest.approx(0.0)
    payload = json.loads(lerobot_path.read_text())
    assert "gripper" in payload
    assert "so101_gripper" not in payload
