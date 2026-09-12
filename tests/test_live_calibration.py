import pytest

from soarm101_motion.calibration_live import (
    EncoderSweep,
    calibration_from_sweeps,
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
        name: sweep([3500, 3900, 100, 500, 700, 100, 3900, 3500])
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
