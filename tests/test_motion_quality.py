from __future__ import annotations

import json

import pytest

from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.motion.quality import (
    command_sequence,
    load_jsonl,
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
