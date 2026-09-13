import pytest

from soarm101_motion.config import SOARM101Config
from soarm101_motion.constants import ALL_MOTORS
from soarm101_motion.exceptions import SafetyViolationError
from soarm101_motion.hardware.managed_feetech import FeetechBackend


class FakeEffortBackend(FeetechBackend):
    def __init__(self, config: SOARM101Config) -> None:
        super().__init__(config)
        self._connected = True
        self._torque_enabled = True
        self.currents = dict.fromkeys(ALL_MOTORS, 0)
        self.loads = dict.fromkeys(ALL_MOTORS, 0)
        self.hold_count = 0

    def read_register(self, motor: str, register: str) -> int:
        if register == "Moving":
            return 1
        if register == "Status":
            return 0
        if register == "Present_Current":
            return int(self.currents[motor])
        if register == "Present_Load":
            return int(self.loads[motor])
        raise AssertionError(f"unexpected register read: {register}")

    def read_raw_position(self, motor: str) -> int:
        del motor
        return 2047

    def _write_raw_positions(self, positions, *, speed_raw: int, acceleration_raw: int) -> None:
        del positions, speed_raw, acceleration_raw
        self.hold_count += 1


def test_present_load_sign_magnitude_decoding() -> None:
    assert FeetechBackend._decode_present_load(500) == 500
    assert FeetechBackend._decode_present_load(0x0400 | 500) == -500


def test_effort_threshold_holds_and_latches_after_required_samples() -> None:
    config = SOARM101Config(
        port="fake",
        use_stored_calibration=False,
        effort_current_trip_raw=100,
        effort_load_trip_raw=None,
        effort_trip_consecutive_samples=2,
    )
    backend = FakeEffortBackend(config)
    backend.currents["elbow_flex"] = 120

    first = backend.get_hardware_state()
    assert not first.faulted
    assert backend.hold_count == 0

    second = backend.get_hardware_state()
    assert second.faulted
    assert not second.moving
    assert "elbow_flex" in (second.fault_message or "")
    assert "current 120 >= 100" in (second.fault_message or "")
    assert backend.hold_count == 1

    # The interlock stays latched even after current falls until explicitly cleared.
    backend.currents["elbow_flex"] = 0
    latched = backend.get_hardware_state()
    assert latched.faulted
    assert backend.hold_count == 1

    with pytest.raises(SafetyViolationError, match="clear_effort_trip"):
        backend._require_effort_clear()

    backend.clear_effort_trip()
    backend._require_effort_clear()
    cleared = backend.get_hardware_state()
    assert not cleared.faulted


def test_per_motor_load_override_can_be_more_sensitive() -> None:
    config = SOARM101Config(
        port="fake",
        use_stored_calibration=False,
        effort_current_trip_raw=None,
        effort_load_trip_raw=900,
        motor_load_trip_raw={"so101_gripper": 400},
        effort_trip_consecutive_samples=1,
    )
    backend = FakeEffortBackend(config)
    backend.loads["so101_gripper"] = 450

    state = backend.get_hardware_state()
    assert state.faulted
    assert "so101_gripper" in (state.fault_message or "")
    assert "|load| 450 >= 400" in (state.fault_message or "")
    assert backend.hold_count == 1
