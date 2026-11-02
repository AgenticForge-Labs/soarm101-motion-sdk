"""Hardware-free regression tests for the passive wobble trace analyzer."""
from __future__ import annotations

import json
import runpy
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "analyze_wobble_traces.py"
functions = runpy.run_path(str(SCRIPT))
analyze = functions["analyze"]
run_lengths = functions["run_lengths"]


def _event(kind, at, raw=None, radians=None, **extra):
    result = {"event": kind, "monotonic_s": at, **extra}
    if raw is not None:
        result["joints_raw"] = {"shoulder_pan": raw}
    if radians is not None:
        result["joints_rad"] = {"shoulder_pan": radians}
    return result


def _write(path, events):
    path.write_text("".join(json.dumps(e) + "\n" for e in events), encoding="utf-8")


def test_zero_runs_are_increments_not_sample_counts():
    assert run_lengths([4, 4, 4, 5, 5, 7]) == [2, 1]
    assert run_lengths([4, 5]) == []
    assert run_lengths([4, 4, 4]) == [2]


def test_silent_stretch_and_tail_impulse_are_recovered_from_sparse_writes(tmp_path):
    path = tmp_path / "planned.jsonl"
    events = [
        {"event": "trace_start", "monotonic_s": 0, "command_frequency_hz": 50,
         "kind": "program_motion_quality", "execution_mode": "streamed"},
        {"event": "marker", "monotonic_s": 0, "marker": "leg_start", "destination": "SLEEP"},
        _event("command", 0.0, 2000, 0.0, speed_raw={"shoulder_pan": 0}),
        _event("feedback", 0.02, 2000, 0.0),
        _event("feedback", 0.06, 2001, 0.00153),
        _event("feedback", 0.10, 2000, 0.0),
        _event("feedback", 0.14, 2001, 0.00153),
        _event("feedback", 0.18, 2001, 0.00153),
        _event("command", 0.20, 2001, 0.00153, speed_raw={"shoulder_pan": 63}),
        {"event": "marker", "monotonic_s": 0.22, "marker": "leg_end", "destination": "SLEEP"},
    ]
    _write(path, events)
    output = analyze(path, "streamed", None, None, False, None, 0.5)
    assert len(output) == 1
    pan = output[0]["joints"][0]
    assert pan["max_zero_cycles"] >= 8
    assert pan["tick_impulses_final25"] == 1
    assert pan["tail30_mean_abs_ticks_per_cycle"] == 0.33333
    assert pan["tail30_max_abs_ticks_per_cycle"] == 1
    assert pan["tail30_zero_cycles_fraction"] == 0.66667
    assert pan["hunting_reversals"] == 2
    assert pan["following_rms"] is not None
    assert pan["subtick_tail_ms_estimate"] == 200.0
    assert pan["servo_speed_raw_min"] == 0
    assert pan["servo_speed_raw_max"] == 63
    assert pan["servo_speed_raw_changes"] == 1
    assert pan["post_gap_speed_raw_max"] == 63
    assert pan["post_gap_speed_raw_samples"] == 1
    gripper = output[0]["joints"][5]
    assert gripper["max_zero_cycles"] is None
    assert "no commands for joint" in gripper["notes"]


def test_sleep2_is_separate_segment_and_one_shot_is_not_misclassified(tmp_path):
    path = tmp_path / "one_shot.jsonl"
    _write(path, [
        {"event": "trace_start", "monotonic_s": 0, "command_frequency_hz": 50,
         "execution_mode": "final_target"},
        {"event": "marker", "monotonic_s": 0, "marker": "condition_start",
         "condition": "sleep2"},
        _event("command", 0.02, 2000, 0.0),
        _event("feedback", 0.06, 1990, -0.01534),
        _event("feedback", 0.10, 1998, -0.00307),
        {"event": "marker", "monotonic_s": 0.20, "marker": "condition_end",
         "condition": "sleep2"},
    ])
    result = analyze(path, "one-shot", None, None, False, "sleep2", 0.5)
    assert len(result) == 1
    assert result[0]["segment"].startswith("sleep2")
    pan = result[0]["joints"][0]
    assert pan["max_zero_cycles"] is None
    assert pan["following_peak"] is not None
    assert pan["lag_ms"] is None


def test_gui_teleop_frames_and_real_gripper_mapping(tmp_path):
    path = tmp_path / "teleop.jsonl"
    frames = [{"event": "trace_start", "monotonic_s": 0, "kind": "exact_teleop_command_replay"}]
    for i in range(5):
        frames.append({
            "event": "teleop_frame",
            "monotonic_s": i * 0.05,
            "frequency_hz": 20,
            "command_joints_raw": {"shoulder_pan": 2000+i},
            "actual_joints_raw": {"shoulder_pan": 1999+i},
            "command_joints_rad": {"shoulder_pan": i * 0.00153},
            "actual_joints_rad": {"shoulder_pan": (i-1) * 0.00153},
        })
    _write(path, frames)
    result = analyze(path, "teleop", None, (1000, 3000), False, None, 0.5)
    assert result[0]["frequency_hz"] == 20
    assert result[0]["joints"][0]["following_rms"] is not None
    assert result[0]["joints"][5]["max_zero_cycles"] is None
