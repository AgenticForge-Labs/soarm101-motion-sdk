import json
import pytest

from soarm101_motion.calibration import MotorCalibration, SO101Calibration
from soarm101_motion.constants import ALL_MOTORS, MOTOR_IDS, STOCK_GRIPPER


def sample_calibration() -> SO101Calibration:
    return SO101Calibration(
        motors={
            name: MotorCalibration(MOTOR_IDS[name], 0, 0, 100, 3995)
            for name in ALL_MOTORS
        }
    )


def test_joint_calibration_round_trip() -> None:
    calibration = MotorCalibration(1, 0, 0, 100, 3995)
    for radians in (-1.0, 0.0, 1.0):
        recovered = calibration.raw_to_radians(calibration.radians_to_raw(radians))
        assert recovered == pytest.approx(radians, abs=0.002)


def test_gripper_calibration_round_trip_and_inversion() -> None:
    normal = MotorCalibration(6, 0, 0, 100, 1100)
    inverted = MotorCalibration(6, 1, 0, 100, 1100)
    assert normal.raw_to_normalized(100) == 0.0
    assert normal.raw_to_normalized(1100) == 1.0
    assert inverted.raw_to_normalized(100) == 1.0
    assert inverted.normalized_to_raw(0.25) == 850


def test_load_and_export_lerobot_format(tmp_path) -> None:
    calibration = sample_calibration()
    own_path = tmp_path / "calibration.json"
    lerobot_path = tmp_path / "lerobot.json"
    calibration.save(own_path)
    calibration.save_lerobot(lerobot_path)

    assert SO101Calibration.load(own_path).motors.keys() == calibration.motors.keys()
    loaded = SO101Calibration.load(lerobot_path)
    assert STOCK_GRIPPER in loaded.motors
    payload = json.loads(lerobot_path.read_text())
    assert "gripper" in payload
    assert "so101_gripper" not in payload
