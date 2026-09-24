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


def test_contact_latched_gripper_effort_does_not_stop_arm_stream() -> None:
    config = SOARM101Config(
        port="fake",
        use_stored_calibration=False,
        effort_current_trip_raw=100,
        effort_load_trip_raw=None,
        effort_trip_consecutive_samples=1,
    )
    backend = FakeEffortBackend(config)
    backend.currents["so101_gripper"] = 150
    backend.set_gripper_contact_latched(True)

    state = backend.get_hardware_state()

    assert not state.faulted
    assert backend.hold_count == 0
    assert "so101_gripper" not in backend._last_effort_readings

    backend.currents["wrist_flex"] = 150
    state = backend.get_hardware_state()
    assert state.faulted
    assert "wrist_flex" in (state.fault_message or "")
    assert backend.hold_count == 1


def test_effort_status_reports_live_readings_peaks_and_effective_limits() -> None:
    config = SOARM101Config(
        port="fake",
        use_stored_calibration=False,
        effort_current_trip_raw=250,
        effort_load_trip_raw=850,
        motor_current_trip_raw={"wrist_roll": 180},
    )
    backend = FakeEffortBackend(config)
    backend.currents["wrist_roll"] = 120
    backend.loads["wrist_roll"] = 0x0400 | 300

    state = backend.get_hardware_state()
    assert not state.faulted

    status = backend.get_effort_safety_status()
    assert status["supported"] is True
    assert status["enabled"] is True
    assert status["readings"]["wrist_roll"] == {
        "current_raw": 120,
        "load_raw": -300,
    }
    assert status["peaks"]["wrist_roll"] == {
        "current_raw": 120,
        "abs_load_raw": 300,
    }
    assert status["effective_limits"]["wrist_roll"]["current_raw"] == 180
    assert status["effective_limits"]["wrist_roll"]["load_raw"] == 850

    backend.currents["wrist_roll"] = 150
    backend.loads["wrist_roll"] = 100
    backend.get_hardware_state()
    status = backend.get_effort_safety_status()
    assert status["peaks"]["wrist_roll"]["current_raw"] == 150
    assert status["peaks"]["wrist_roll"]["abs_load_raw"] == 300

    backend.reset_effort_peaks()
    status = backend.get_effort_safety_status()
    assert status["peaks"]["wrist_roll"] == {
        "current_raw": 0,
        "abs_load_raw": 0,
    }


def test_effort_settings_can_only_change_with_torque_off() -> None:
    backend = FakeEffortBackend(
        SOARM101Config(port="fake", use_stored_calibration=False)
    )

    with pytest.raises(SafetyViolationError, match="disable torque"):
        backend.configure_effort_safety(
            enabled=False,
            current_trip_raw=None,
            load_trip_raw=500,
            consecutive_samples=3,
        )

    backend._torque_enabled = False
    backend.configure_effort_safety(
        enabled=False,
        current_trip_raw=None,
        load_trip_raw=500,
        consecutive_samples=3,
    )
    status = backend.get_effort_safety_status()
    assert status["enabled"] is False
    assert status["current_trip_raw"] is None
    assert status["load_trip_raw"] == 500
    assert status["consecutive_samples"] == 3


def test_changing_effort_settings_does_not_clear_latched_trip() -> None:
    backend = FakeEffortBackend(
        SOARM101Config(port="fake", use_stored_calibration=False)
    )
    backend._effort_trip_message = "motor effort safety trip: test"
    backend._torque_enabled = False

    backend.configure_effort_safety(
        enabled=True,
        current_trip_raw=300,
        load_trip_raw=900,
        consecutive_samples=4,
    )

    assert backend.effort_trip_message == "motor effort safety trip: test"
    with pytest.raises(SafetyViolationError, match="clear_effort_trip"):
        backend._require_effort_clear()


def test_manual_effort_refresh_updates_cache_without_trip_evaluation() -> None:
    backend = FakeEffortBackend(
        SOARM101Config(
            port="fake",
            use_stored_calibration=False,
            effort_current_trip_raw=100,
            effort_trip_consecutive_samples=1,
        )
    )
    backend._torque_enabled = False
    backend.currents["elbow_flex"] = 150

    status = backend.get_effort_safety_status(refresh=True)

    assert status["readings"]["elbow_flex"]["current_raw"] == 150
    assert status["trip_message"] is None
