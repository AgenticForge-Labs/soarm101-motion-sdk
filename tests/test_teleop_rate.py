"""Live teleoperation limits abrupt leader samples before they reach the arm."""

from math import degrees, radians

import pytest

from soarm101_motion import SOARM101, SOARM101Config
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.constants import (
    TELEOP_SERVO_ACCELERATION_RAW,
    TELEOP_SERVO_SPEED_RAW,
)
from soarm101_motion.gui.teleop_rate import (
    GripperContactLatch,
    limit_joint_target,
    plan_alignment_target,
    update_gripper_contact_latch,
)
from soarm101_motion.hardware import SimulationBackend


class RecordingSimulationBackend(SimulationBackend):
    def __init__(self) -> None:
        super().__init__(realtime=False)
        self.last_stream_profile: tuple[int | None, int | None] | None = None

    def write_joint_positions(
        self,
        positions,
        *,
        speed_raw=None,
        acceleration_raw=None,
    ) -> None:
        self.last_stream_profile = (speed_raw, acceleration_raw)
        super().write_joint_positions(
            positions,
            speed_raw=speed_raw,
            acceleration_raw=acceleration_raw,
        )


def test_alignment_accepts_recorded_wrist_travel_beyond_model_limit() -> None:
    leader = {name: 0.0 for name in ARM_JOINTS}
    leader.update(shoulder_lift=radians(-103.4), wrist_flex=radians(-101.2))
    motion = {name: (radians(-100.0), radians(100.0)) for name in ARM_JOINTS}
    motion["wrist_flex"] = (radians(-95.0), radians(95.0))
    calibrated = dict(motion)
    calibrated["shoulder_lift"] = (radians(-105.1), radians(105.1))
    calibrated["wrist_flex"] = (radians(-104.0), radians(104.0))

    target, offsets = plan_alignment_target(leader, motion, calibrated)

    assert degrees(target["shoulder_lift"]) == pytest.approx(-99.5)
    assert degrees(target["wrist_flex"]) == pytest.approx(-94.5)
    assert set(offsets) == {"shoulder_lift", "wrist_flex"}
    assert degrees(offsets["wrist_flex"]) == pytest.approx(6.7)


def test_alignment_rejects_pose_beyond_follower_calibration() -> None:
    leader = {name: 0.0 for name in ARM_JOINTS}
    leader["wrist_flex"] = radians(105.0)
    limits = {name: (radians(-95.0), radians(95.0)) for name in ARM_JOINTS}
    calibrated = dict(limits, wrist_flex=(radians(-104.0), radians(104.0)))
    with pytest.raises(ValueError, match="outside the follower's calibrated travel"):
        plan_alignment_target(leader, limits, calibrated)


def test_abrupt_leader_motion_is_smoothed_within_stream_limits() -> None:
    previous = {name: 0.0 for name in ARM_JOINTS}
    velocity = dict(previous)
    desired = dict(previous, shoulder_pan=0.3)
    limited_count = 0
    for _ in range(8):
        command, new_velocity, limited = limit_joint_target(
            desired,
            previous,
            velocity,
            period_s=0.1,
            max_speed_rad_s=1.0,
            max_acceleration_rad_s2=5.0,
            max_step_rad=0.087,
        )
        assert abs(command["shoulder_pan"] - previous["shoulder_pan"]) <= 0.087 + 1e-9
        assert abs(new_velocity["shoulder_pan"]) <= 1.0 + 1e-9
        assert abs(new_velocity["shoulder_pan"] - velocity["shoulder_pan"]) <= 0.5 + 1e-9
        limited_count += int(limited)
        previous, velocity = command, new_velocity
    assert limited_count > 0
    assert abs(previous["shoulder_pan"] - 0.3) < 0.06


def test_steady_leader_target_does_not_make_follower_reverse_past_it() -> None:
    previous = {name: 0.0 for name in ARM_JOINTS}
    velocity = dict(previous)
    desired = dict(previous, shoulder_pan=0.3)
    positions = []
    for _ in range(20):
        command, new_velocity, _ = limit_joint_target(
            desired,
            previous,
            velocity,
            period_s=0.1,
            max_speed_rad_s=1.0,
            max_acceleration_rad_s2=5.0,
            max_step_rad=0.087,
        )
        positions.append(command["shoulder_pan"])
        assert abs(new_velocity["shoulder_pan"] - velocity["shoulder_pan"]) <= 0.5 + 1e-8
        previous, velocity = command, new_velocity
    assert all(a <= b + 1e-8 for a, b in zip(positions, positions[1:]))
    assert max(positions) <= 0.3 + 1e-8
    assert abs(positions[-1] - 0.3) < 1e-6


def test_outward_leader_target_holds_at_follower_joint_limit() -> None:
    lower = radians(-100.0)
    previous = {name: 0.0 for name in ARM_JOINTS}
    previous["shoulder_lift"] = lower
    velocity = dict.fromkeys(ARM_JOINTS, 0.0)
    desired = dict(previous, shoulder_lift=lower - radians(0.1))
    limits = {name: (radians(-100.0), radians(100.0)) for name in ARM_JOINTS}

    command, _, limited = limit_joint_target(
        desired,
        previous,
        velocity,
        joint_limits=limits,
        period_s=0.05,
        max_speed_rad_s=1.0,
        max_acceleration_rad_s2=5.0,
        max_step_rad=radians(5.0),
    )

    assert limited
    assert command["shoulder_lift"] == pytest.approx(lower)


def test_gripper_contact_latches_aperture_until_leader_opens() -> None:
    latch = GripperContactLatch(last_command=0.3, last_actual=0.3)
    for _ in range(5):
        command, active, newly_latched, released = update_gripper_contact_latch(
            latch, 0.0, 0.3, period_s=0.05
        )
        assert not active and not newly_latched and not released
    command, active, newly_latched, released = update_gripper_contact_latch(
        latch, 0.0, 0.3, period_s=0.05
    )
    assert command == pytest.approx(0.305)
    assert active and newly_latched and not released

    command, active, newly_latched, released = update_gripper_contact_latch(
        latch, 0.0, 0.3, period_s=0.05
    )
    assert command == pytest.approx(0.305)
    assert active and not newly_latched and not released

    command, active, newly_latched, released = update_gripper_contact_latch(
        latch, 0.05, 0.3, period_s=0.05
    )
    assert command == pytest.approx(0.355)
    assert not active and not newly_latched and released


def test_gripper_default_ramp_is_slightly_faster_without_skipping_contact_guard() -> None:
    latch = GripperContactLatch(last_command=0.8, last_actual=0.8)
    command, active, newly_latched, _ = update_gripper_contact_latch(
        latch, 0.0, 0.8, period_s=0.05
    )
    assert command == pytest.approx(0.74)
    assert not active and not newly_latched
    for _ in range(5):
        command, active, newly_latched, _ = update_gripper_contact_latch(
            latch, 0.0, 0.8, period_s=0.05
        )
    assert command == pytest.approx(0.805)
    assert active and newly_latched


def test_gripper_contact_does_not_latch_while_follower_is_catching_up() -> None:
    latch = GripperContactLatch(last_command=0.54, last_actual=0.54)
    # This is the closing reversal from the physical session: feedback lagged
    # the leader target, but the follower was still making closing progress.
    actuals = [0.538, 0.538, 0.534, 0.528, 0.524, 0.517, 0.508, 0.499]
    for actual in actuals:
        _, active, newly_latched, _ = update_gripper_contact_latch(
            latch, 0.0, actual, period_s=0.05
        )
        assert not active and not newly_latched


def test_gripper_contact_does_not_latch_during_open_to_close_reversal() -> None:
    latch = GripperContactLatch(last_command=0.838, last_actual=0.724)
    # The real follower was still opening for several samples after the
    # leader reversed. It then began closing; neither phase is contact.
    samples = [
        (0.669, 0.733), (0.564, 0.743), (0.498, 0.750),
        (0.511, 0.757), (0.524, 0.762), (0.524, 0.762),
        (0.524, 0.761), (0.524, 0.754), (0.524, 0.748),
        (0.524, 0.742), (0.524, 0.738),
    ]
    for desired, actual in samples:
        _, active, newly_latched, _ = update_gripper_contact_latch(
            latch, desired, actual, period_s=0.05
        )
        assert not active and not newly_latched


def test_gripper_contact_releases_when_leader_range_is_below_follower_aperture() -> None:
    latch = GripperContactLatch(last_command=0.54, last_actual=0.54)
    for _ in range(6):
        update_gripper_contact_latch(latch, 0.42, 0.54, period_s=0.05)
    assert latch.contact_position == pytest.approx(0.545)

    command, active, _, released = update_gripper_contact_latch(
        latch, 0.47, 0.54, period_s=0.05
    )
    assert released and not active
    assert command > 0.54

    # A new close gesture must work after the leader reverses.
    update_gripper_contact_latch(latch, 0.50, 0.56, period_s=0.05)
    for leader_position in (0.495, 0.49, 0.485, 0.48):
        update_gripper_contact_latch(latch, leader_position, 0.56, period_s=0.05)
    command, active, _, _ = update_gripper_contact_latch(
        latch, 0.475, 0.56, period_s=0.05
    )
    assert command < 0.6
    assert not active


def test_gripper_release_target_stays_within_normalized_range() -> None:
    latch = GripperContactLatch(last_command=0.95, last_actual=0.95)
    for _ in range(6):
        update_gripper_contact_latch(latch, 0.8, 0.95, period_s=0.05)
    assert latch.contact_position is not None

    for desired in (0.9, 1.0, 1.0):
        command, _, _, _ = update_gripper_contact_latch(
            latch, desired, 0.95, period_s=0.05
        )
        assert 0.0 <= command <= 1.0


def test_empty_close_stops_short_of_calibrated_hard_stop() -> None:
    latch = GripperContactLatch(last_command=0.10, last_actual=0.10)
    for actual in (0.10, 0.06, 0.03, 0.025, 0.025, 0.025):
        command, _, _, _ = update_gripper_contact_latch(
            latch, 0.0, actual, period_s=0.05
        )
        assert command >= 0.025


@pytest.mark.parametrize("frequency_hz", [10.0, 20.0])
def test_smoothed_targets_pass_the_stream_guard_in_simulation(frequency_hz: float) -> None:
    config = SOARM101Config(enable_workspace_checks=False, effort_safety_enabled=False)
    arm = SOARM101(config, backend=SimulationBackend(realtime=False))
    arm.connect()
    arm.enable()
    try:
        arm.start_joint_stream(frequency_hz=frequency_hz)
        previous = dict(arm.get_joint_positions().positions)
        velocity = {name: 0.0 for name in ARM_JOINTS}
        desired = dict(previous, shoulder_pan=previous["shoulder_pan"] + 0.3)
        for _ in range(40):
            command, velocity, _ = limit_joint_target(
                desired,
                previous,
                velocity,
                period_s=1.0 / frequency_hz,
                max_speed_rad_s=config.max_joint_speed,
                max_acceleration_rad_s2=config.max_joint_acceleration,
                max_step_rad=config.max_command_step_radians,
            )
            assert arm.stream_joint_target(command).accepted
            previous = command
        assert abs(previous["shoulder_pan"] - desired["shoulder_pan"]) < 1e-6
    finally:
        arm.stop_joint_stream(hold=True)
        arm.disconnect()


def test_stream_uses_unrestricted_feetech_speed_and_le_robot_acceleration() -> None:
    backend = RecordingSimulationBackend()
    arm = SOARM101(
        SOARM101Config(enable_workspace_checks=False, effort_safety_enabled=False),
        backend=backend,
    )
    arm.connect()
    arm.enable()
    try:
        arm.start_joint_stream(frequency_hz=20.0)
        target = dict(arm.get_joint_positions().positions)
        target["wrist_flex"] += 0.01
        arm.stream_joint_target(target)
        assert backend.last_stream_profile == (
            TELEOP_SERVO_SPEED_RAW,
            TELEOP_SERVO_ACCELERATION_RAW,
        )
    finally:
        arm.stop_joint_stream(hold=True)
        arm.disconnect()
