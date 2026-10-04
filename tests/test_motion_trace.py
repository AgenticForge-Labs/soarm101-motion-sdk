from __future__ import annotations

import json
from types import SimpleNamespace

from soarm101_motion import SOARM101
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.motion import PassiveBackendTrace


class _FakeMotor:
    radians_limits = (-3.0, 3.0)

    @staticmethod
    def radians_to_raw(value: float) -> int:
        return int(round(2048 + float(value) * 1000.0))


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
