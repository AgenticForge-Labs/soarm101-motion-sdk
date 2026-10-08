from __future__ import annotations

import json
from types import SimpleNamespace

from soarm101_motion import SOARM101
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.motion import PassiveBackendTrace
from soarm101_motion.motion.trace import summarize_agent_jog_trace


class _FakeMotor:
    radians_limits = (-3.0, 3.0)

    @staticmethod
    def radians_to_raw(value: float) -> int:
        return int(round(2048 + float(value) * 1000.0))


def test_passive_backend_trace_reserves_runtime_provenance_fields(tmp_path) -> None:
    path = tmp_path / "provenance.jsonl"

    with SOARM101.simulated() as arm:
        arm.backend.calibration = SimpleNamespace(
            calibration_id="trace-test",
            motors={name: _FakeMotor() for name in ARM_JOINTS},
        )
        arm.enable()
        with PassiveBackendTrace(
            arm,
            path,
            metadata={
                "robot_id": "wrong-robot",
                "calibration_id": "wrong-calibration",
                "test_case": "reserved-provenance",
            },
        ):
            pass

    start = json.loads(path.read_text().splitlines()[0])
    assert start["event"] == "trace_start"
    assert start["robot_id"] == arm.config.robot_id
    assert start["calibration_id"] == arm.calibration_id
    assert start["test_case"] == "reserved-provenance"


def test_passive_backend_trace_records_existing_motion_io(tmp_path) -> None:
    path = tmp_path / "motion.jsonl"

    with SOARM101.simulated() as arm:
        arm.backend.calibration = SimpleNamespace(
            calibration_id="trace-test",
            motors={name: _FakeMotor() for name in ARM_JOINTS},
        )
        arm.enable()
        target = dict(arm.get_joint_positions().positions)
        target["shoulder_pan"] += 0.04

        with PassiveBackendTrace(arm, path, metadata={"test_case": "unit"}) as trace:
            result = arm.move_joints(
                target,
                speed=0.20,
                acceleration=0.50,
                workspace_check="off",
            )
            trace.mark("complete", completed=result.completed)

        summary = dict(trace.summary)

    events = [json.loads(line) for line in path.read_text().splitlines()]
    kinds = [event["event"] for event in events]

    assert events[0]["event"] == "trace_start"
    assert events[0]["test_case"] == "unit"
    assert "command" in kinds
    assert "feedback" in kinds
    assert "hardware_state" in kinds
    assert "marker" in kinds
    assert events[-1]["event"] == "trace_end"
    assert summary["command_count"] > 0
    assert summary["feedback_count"] > 0

    command = next(event for event in events if event["event"] == "command")
    feedback = next(event for event in events if event["event"] == "feedback")
    assert set(command["joints_rad"]) == set(ARM_JOINTS)
    assert set(command["joints_raw"]) == set(ARM_JOINTS)
    assert command["tcp_xyz_mm"] is not None
    assert set(feedback["joints_rad"]) == set(ARM_JOINTS)
    assert set(feedback["joints_raw"]) == set(ARM_JOINTS)
    assert feedback["tcp_xyz_mm"] is not None



def test_passive_trace_captures_raw_hold_latch_without_extra_reads(tmp_path) -> None:
    """Feetech STOP writes raw goals; passive trace must see them."""
    path = tmp_path / "raw-hold.jsonl"
    with SOARM101.simulated() as arm:
        arm.enable()
        received = []

        def fake_raw_write(positions, *, speed_raw, acceleration_raw):
            received.append((dict(positions), speed_raw, acceleration_raw))

        arm.backend._write_raw_positions = fake_raw_write
        raw = {name: 2048 + i for i, name in enumerate(ARM_JOINTS)}
        with PassiveBackendTrace(arm, path) as trace:
            trace.mark("hold_start")
            arm.backend._write_raw_positions(raw, speed_raw=1, acceleration_raw=1)
            trace.mark("hold_complete")

    events = [json.loads(line) for line in path.read_text().splitlines()]
    markers = [e["marker"] for e in events if e["event"] == "marker"]
    writes = [e for e in events if e["event"] == "raw_command"]
    assert markers == ["hold_start", "hold_complete"]
    assert received == [(raw, 1, 1)]
    assert len(writes) == 1
    assert writes[0]["joints_raw"] == raw
    assert writes[0]["speed_raw"] == 1
    assert writes[0]["acceleration_raw"] == 1
    assert not any(e["event"] == "feedback" for e in events)



def test_offline_jog_summary_separates_command_feedback_and_hold(tmp_path) -> None:
    path = tmp_path / "synthetic.jsonl"
    events = [
        {
            "event": "trace_start",
            "robot_id": "so101", "calibration_id": "sha256:known",
            "action": "agent_jog", "frame": "world",
            "delta_model_mm": [2.0, 0.0, 0.0],
            "requested_speed_mm_s": 10.0,
        },
        {
            "event": "marker", "marker": "preflight",
            "start_model_xyz_mm": [195.0, 0.0, 100.0],
            "target_model_xyz_mm": [197.0, 0.0, 100.0],
        },
        {"event": "command", "tcp_xyz_mm": [195.0, 0.0, 100.0]},
        {"event": "raw_command", "joints_raw": {"shoulder_lift": 2000}},
        {"event": "feedback", "tcp_xyz_mm": [195.0, 0.0, 100.0]},
        {"event": "command", "tcp_xyz_mm": [197.0, 0.0, 100.0]},
        {"event": "raw_command", "joints_raw": {"shoulder_lift": 2020}},
        {"event": "feedback", "tcp_xyz_mm": [197.0, 0.0, 97.0]},
        {
            "event": "marker", "marker": "motion_completed_before_hold",
            "completed": True, "tcp_xyz_mm": [197.0, 0.0, 97.0],
        },
        {"event": "marker", "marker": "hold_start"},
        {"event": "raw_command", "joints_raw": {"shoulder_lift": 2010}},
        {"event": "marker", "marker": "hold_complete"},
        {
            "event": "marker", "marker": "post_hold_immediate",
            "tcp_xyz_mm": [197.0, 0.0, 96.0],
        },
        {
            "event": "marker", "marker": "post_hold_2s",
            "tcp_xyz_mm": [197.0, 0.0, 95.5],
        },
        {"event": "trace_end", "error": None},
    ]
    path.write_text("\n".join(json.dumps(e) for e in events) + "\n")
    summary = summarize_agent_jog_trace(path)
    assert summary["completed"] is True
    assert summary["calibration_id"] == "sha256:known"
    assert summary["request"]["target_model_xyz_mm"] == [197.0, 0.0, 100.0]
    assert summary["trajectory"]["joint_commands"] == 2
    assert summary["trajectory"]["commanded_model_z_change_mm"] == 0.0
    assert summary["trajectory"]["observed_model_z_change_mm"] == -3.0
    assert summary["hold"]["hold_goal_delta_ticks"] == {"shoulder_lift": -10}
    assert summary["hold"]["model_z_change_over_2s_after_hold_mm"] == -0.5
    assert "not direct measurements" in summary["coordinate_warning"]


def test_offline_trace_summary_rejects_non_trace_jsonl(tmp_path) -> None:
    path = tmp_path / "unrelated.jsonl"
    path.write_text('{"event": "something_else"}\n')
    import pytest
    with pytest.raises(ValueError, match="not a passive"):
        summarize_agent_jog_trace(path)
