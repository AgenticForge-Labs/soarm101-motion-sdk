from __future__ import annotations

import math

import pytest

from soarm101_motion import SOARM101
from soarm101_motion.gui.manual_follower import JointTargetFollower


def test_follower_shapes_speed_acceleration_and_latest_target_reversal() -> None:
    follower = JointTargetFollower(
        {"shoulder_pan": 0.0}, frequency_hz=20.0,
        max_speed=1.0, max_acceleration=2.0, max_step=0.1,
    )
    target = {"shoulder_pan": 1.0}
    prior_position, prior_velocity = 0.0, 0.0
    for _ in range(8):
        sample = follower.step(target)
        velocity = (sample["shoulder_pan"] - prior_position) * 20.0
        assert abs(velocity) <= 1.0 + 1e-9
        assert abs(velocity - prior_velocity) <= 2.0 / 20.0 + 1e-9
        prior_position, prior_velocity = sample["shoulder_pan"], velocity
    target["shoulder_pan"] = -0.5  # replacement, never queued behind the old target
    prior_position, prior_velocity = follower.position["shoulder_pan"], follower.velocity["shoulder_pan"]
    for _ in range(120):
        sample = follower.step(target)
        velocity = (sample["shoulder_pan"] - prior_position) * 20.0
        assert abs(velocity) <= 1.0 + 1e-9
        assert abs(velocity - prior_velocity) <= 2.0 / 20.0 + 1e-9
        prior_position, prior_velocity = sample["shoulder_pan"], velocity
    assert follower.settled(target)
    assert abs(follower.position["shoulder_pan"] + 0.5) <= 0.001


@pytest.mark.parametrize("distance", [0.002, 0.01, 0.03, 0.2, 0.9])
def test_follower_stops_without_crossing_short_targets_or_oscillating(distance: float) -> None:
    follower = JointTargetFollower(
        {"joint": 0.0}, frequency_hz=20.0, max_speed=1.0,
        max_acceleration=2.0, max_step=0.05,
    )
    target = {"joint": distance}
    prior_position = prior_velocity = 0.0
    positions: list[float] = []
    for _ in range(300):
        position = follower.step(target)["joint"]
        velocity = (position - prior_position) * 20.0
        assert abs(velocity) <= 1.0 + 1e-9
        assert abs(velocity - prior_velocity) <= 0.1 + 1e-9
        positions.append(position)
        prior_position, prior_velocity = position, velocity
        if follower.settled(target):
            break
    assert follower.settled(target)
    assert max(positions) <= distance + 1e-9
    assert positions[-1] == max(positions)


def test_follower_reverses_near_bounds_and_rejects_nonfinite_inputs() -> None:
    follower = JointTargetFollower(
        {"joint": 0.99}, frequency_hz=20.0, max_speed=1.0,
        max_acceleration=2.0, max_step=0.05,
    )
    target = {"joint": 1.0}
    for _ in range(30):
        follower.step(target)
    target["joint"] = 0.95
    samples = [follower.step(target)["joint"] for _ in range(100)]
    assert follower.settled(target)
    assert all(0.0 <= value <= 1.0 for value in samples)
    with pytest.raises(ValueError, match="finite"):
        follower.step({"joint": math.nan})


@pytest.mark.parametrize("speed_deg_s,accel_deg_s2", [(10.0, 100.0), (50.0, 500.0), (100.0, 1000.0)])
def test_follower_samples_pass_real_sdk_simulated_stream_at_fixed_ticks(
    speed_deg_s: float, accel_deg_s2: float
) -> None:
    """Exercise the SDK's real stream validators with a deterministic 20 Hz tick loop."""
    from math import radians

    arm = SOARM101.simulated()
    arm.connect()
    arm.enable()
    start = dict(arm.get_joint_positions().positions)
    frequency = 20.0
    arm.start_joint_stream(
        frequency_hz=frequency,
        max_speed=radians(speed_deg_s),
        max_acceleration=radians(accel_deg_s2),
    )
    follower = JointTargetFollower(
        start,
        frequency_hz=frequency,
        max_speed=radians(speed_deg_s),
        max_acceleration=radians(accel_deg_s2),
        max_step=arm.config.max_command_step_radians,
    )
    limits = arm.get_joint_limits()
    target = dict(start)
    target["shoulder_pan"] += 0.4
    try:
        for requested in (target, {**start, "shoulder_pan": start["shoulder_pan"] - 0.1}):
            for _tick in range(500):
                sample = follower.step(requested)
                assert all(limits[name][0] <= sample[name] <= limits[name][1] for name in start)
                result = arm.stream_joint_target(sample)
                assert result.accepted
                if follower.settled(requested):
                    break
            assert follower.settled(requested)
        assert result.final_positions["shoulder_pan"] == pytest.approx(
            start["shoulder_pan"] - 0.1, abs=0.002
        )
    finally:
        arm.stop_joint_stream(hold=True)
        arm.disconnect()
