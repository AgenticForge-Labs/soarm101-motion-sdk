from __future__ import annotations

import json
import math

import pytest

from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.motion.quality import (
    command_sequence,
    load_jsonl,
    analyze_teleop_frame_continuity,
    preflight_stream_commands,
    recorded_teleop_settings,
    repair_single_missing_teleop_frames,
    summarize_teleop_frames,
    teleop_frames,
    write_frame_extract,
)


def _frame(index: int, raw_offset: int) -> dict[str, object]:
    command = {name: 0.01 * index for name in ARM_JOINTS}
    raw = {name: 2000 + raw_offset for name in ARM_JOINTS}
    actual = {name: value - 0.001 for name, value in command.items()}
    return {
        "event": "teleop_frame",
        "monotonic_s": index * 0.05,
        "interval_ms": None if index == 0 else 50.0,
        "processing_ms": 5.0 + index,
        "sample_age_ms": 1.0,
        "frequency_hz": 20.0,
        "command_joints_rad": command,
        "command_joints_raw": raw,
        "actual_joints_rad": actual,
        "following_error_rad": {
            name: actual[name] - command[name] for name in ARM_JOINTS
        },
    }


def test_motion_quality_summary_and_command_extract(tmp_path) -> None:
    frames = [_frame(0, 0), _frame(1, 0), _frame(2, 2)]
    summary = summarize_teleop_frames(frames)

    assert summary["frame_count"] == 3
    assert summary["duration_s"] == pytest.approx(0.1)
    assert summary["nominal_frequency_hz"] == pytest.approx(20.0)
    assert summary["interval_ms"]["median"] == pytest.approx(50.0)
    assert summary["raw_encoder_steps"]["transition_count"] == 2
    assert summary["raw_encoder_steps"]["zero_complete_target_fraction"] == pytest.approx(0.5)
    assert summary["raw_encoder_steps"]["all_joints_median_abs_ticks"] == pytest.approx(1.0)

    sequence = command_sequence(frames)
    assert len(sequence) == 3
    assert set(sequence[0]) == set(ARM_JOINTS)

    path = write_frame_extract(frames, tmp_path / "frames.jsonl")
    rows = load_jsonl(path)
    assert teleop_frames(rows) == frames


def test_command_sequence_rejects_incomplete_frame() -> None:
    with pytest.raises(ValueError, match="complete command_joints_rad"):
        command_sequence([{"event": "teleop_frame", "command_joints_rad": {"shoulder_pan": 0.0}}])


def test_load_jsonl_rejects_non_object(tmp_path) -> None:
    path = tmp_path / "bad.jsonl"
    path.write_text(json.dumps([1, 2, 3]) + "\n", encoding="utf-8")

    with pytest.raises(ValueError, match="not a JSON object"):
        load_jsonl(path)


def test_preflight_stream_commands_accepts_bounded_sequence() -> None:
    start = {name: 0.0 for name in ARM_JOINTS}
    commands = [
        {name: 0.0 for name in ARM_JOINTS},
        {name: 0.01 for name in ARM_JOINTS},
        {name: 0.02 for name in ARM_JOINTS},
    ]
    metrics = preflight_stream_commands(
        commands,
        start=start,
        frequency_hz=20.0,
        max_command_step_radians=0.2,
        max_joint_speed=1.0,
        max_joint_acceleration=10.0,
        limits={name: (-1.0, 1.0) for name in ARM_JOINTS},
    )
    assert metrics["max_step_deg"] == pytest.approx(math.degrees(0.01))
    assert metrics["max_speed_deg_s"] == pytest.approx(math.degrees(0.2))


def test_preflight_stream_commands_rejects_unsafe_start_step() -> None:
    start = {name: 0.0 for name in ARM_JOINTS}
    command = {name: 0.0 for name in ARM_JOINTS}
    command["shoulder_pan"] = 0.3

    with pytest.raises(ValueError, match="step"):
        preflight_stream_commands(
            [command],
            start=start,
            frequency_hz=20.0,
            max_command_step_radians=0.1,
            max_joint_speed=10.0,
            max_joint_acceleration=100.0,
            limits={name: (-1.0, 1.0) for name in ARM_JOINTS},
        )


def test_repair_single_missing_teleop_frame() -> None:
    period = 0.05
    first = _frame(10, 0)
    first["sample"] = 10
    first["leader_timestamp"] = 100.0
    first["command_joints_rad"] = {name: 0.0 for name in ARM_JOINTS}
    first["command_velocity_rad_s"] = {name: 0.0 for name in ARM_JOINTS}

    third = _frame(12, 2)
    third["sample"] = 12
    third["leader_timestamp"] = 100.1
    third["command_joints_rad"] = {name: 0.02 for name in ARM_JOINTS}
    third["command_velocity_rad_s"] = {name: 0.2 for name in ARM_JOINTS}

    before = analyze_teleop_frame_continuity([first, third], frequency_hz=20.0)
    repaired, repairs = repair_single_missing_teleop_frames(
        [first, third],
        frequency_hz=20.0,
    )
    after = analyze_teleop_frame_continuity(repaired, frequency_hz=20.0)

    assert before["gap_count"] == 1
    assert repairs == [{"missing_sample": 11, "source_sample": 12}]
    assert [frame["sample"] for frame in repaired] == [10, 11, 12]
    assert repaired[1]["leader_timestamp"] == pytest.approx(100.1 - period)
    assert repaired[1]["command_joints_rad"]["shoulder_pan"] == pytest.approx(0.01)
    assert after["gap_count"] == 0
    assert after["velocity_mismatch_count"] == 0


def test_larger_teleop_gap_is_not_inferred() -> None:
    first = _frame(20, 0)
    first["sample"] = 20
    second = _frame(23, 3)
    second["sample"] = 23

    repaired, repairs = repair_single_missing_teleop_frames(
        [first, second],
        frequency_hz=20.0,
    )
    continuity = analyze_teleop_frame_continuity(repaired, frequency_hz=20.0)

    assert repairs == []
    assert continuity["gap_count"] == 1


def test_recorded_teleop_settings_uses_latest_session_values() -> None:
    rows = [
        {"event": "session_start"},
        {
            "event": "teleop_settings",
            "max_joint_speed_rad_s": 1.0,
            "max_joint_acceleration_rad_s2": 5.0,
            "max_command_step_rad": 0.10,
            "following_error_limit_rad": 0.20,
        },
        {
            "event": "teleop_settings",
            "max_joint_speed_rad_s": 1.2,
            "max_joint_acceleration_rad_s2": 6.0,
            "max_command_step_rad": 0.12,
            "following_error_limit_rad": 0.30,
        },
    ]
    settings = recorded_teleop_settings(rows)
    assert settings["max_joint_speed_rad_s"] == pytest.approx(1.2)
    assert settings["max_joint_acceleration_rad_s2"] == pytest.approx(6.0)
    assert settings["max_command_step_rad"] == pytest.approx(0.12)
    assert settings["following_error_limit_rad"] == pytest.approx(0.30)


def test_recorded_teleop_settings_rejects_missing_log_event() -> None:
    with pytest.raises(ValueError, match="no teleop_settings"):
        recorded_teleop_settings([{"event": "session_start"}])
