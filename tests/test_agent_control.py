from __future__ import annotations

import json

from types import SimpleNamespace

import numpy as np
import pytest

from soarm101_motion.agent_control import (
    AgentAuthorityStore,
    evaluate_agent_jog,
)
from soarm101_motion.cli.main import _agent_requested_rate, build_parser
from soarm101_motion.workspace import fit_paper_workspace


def _identity_workspace(*, calibration_id: str = "sha256:motor"):
    width = 0.2159
    height = 0.2794
    return fit_paper_workspace(
        {
            "A": (0.0, 0.0, 0.0),
            "B": (width, 0.0, 0.0),
            "C": (width, height, 0.0),
            "D": (0.0, height, 0.0),
        },
        (0.0, height, 0.050),
        robot_id="so101",
        arm_calibration_id=calibration_id,
        width_m=width,
        height_m=height,
        reference_height_m=0.050,
    )


def test_agent_cli_rates_are_explicit_and_validated_before_hardware() -> None:
    parser = build_parser()
    jog = parser.parse_args([
        "agent", "jog", "--frame", "world", "--x-mm", "5",
        "--speed-mm-s", "20", "--acceleration-mm-s2", "80",
    ])
    assert jog.speed_mm_s == 20
    assert jog.acceleration_mm_s2 == 80
    joint = parser.parse_args([
        "agent", "joint", "shoulder_pan", "--delta-deg", "2",
        "--speed-deg-s", "16", "--acceleration-deg-s2", "50",
    ])
    assert joint.speed_deg_s == 16
    assert joint.acceleration_deg_s2 == 50
    assert _agent_requested_rate(20.0, "Cartesian speed", 50.0) == 20.0
    for invalid in [0.0, -1.0, float("nan"), float("inf"), 51.0]:
        with pytest.raises(ValueError):
            _agent_requested_rate(invalid, "Cartesian speed", 50.0)


def test_agent_joint_executes_at_broker_supplied_rates(monkeypatch) -> None:
    from soarm101_motion.cli import main as cli

    class FakeArm:
        def __init__(self):
            self.motion = None

        def __enter__(self):
            return self

        def __exit__(self, *_):
            return False

        def get_joint_positions(self):
            return SimpleNamespace(positions={"shoulder_pan": 0.0})

        def enable(self):
            pass

        def hold(self):
            pass

        def move_joints(self, *args, **kwargs):
            self.motion = kwargs
            return SimpleNamespace(accepted=True, completed=True, message=None)

    arm = FakeArm()
    monkeypatch.setattr(cli, "_arm_from_args", lambda *a, **kw: arm)
    monkeypatch.setattr(cli, "_agent_require_authority", lambda *a: {"armed": True})

    args = build_parser().parse_args([
        "agent", "joint", "shoulder_pan", "--delta-deg", "2",
        "--speed-deg-s", "16", "--acceleration-deg-s2", "50",
    ])
    assert cli._cmd_agent_joint(args) == 0
    assert arm.motion is not None
    assert arm.motion["speed"] == pytest.approx(np.deg2rad(16))
    assert arm.motion["acceleration"] == pytest.approx(np.deg2rad(50))


def test_agent_cartesian_executes_at_broker_supplied_rates(monkeypatch) -> None:
    from soarm101_motion.cli import main as cli

    class FakeArm:
        config = SimpleNamespace(robot_id="so101")

        def __enter__(self):
            return self

        def __exit__(self, *_):
            return False

        def get_position(self):
            return SimpleNamespace(position=np.array([0.0, 0.0, 0.15]))

        def enable(self):
            pass

        def hold(self):
            pass

    captured = {}
    arm = FakeArm()
    monkeypatch.setattr(cli, "_arm_from_args", lambda *a, **kw: arm)
    monkeypatch.setattr(cli, "_agent_require_authority", lambda *a: {"armed": True})
    monkeypatch.setattr(cli, "_agent_calibration_id", lambda *a, **kw: "sha256:motor")
    monkeypatch.setattr(
        cli, "WorkspaceCalibrationStore",
        lambda *a: SimpleNamespace(load=lambda: object()),
    )
    monkeypatch.setattr(
        cli, "relative_target_pose",
        lambda *a, **kw: SimpleNamespace(position=np.array([0.005, 0.0, 0.15])),
    )
    monkeypatch.setattr(
        cli, "evaluate_agent_jog",
        lambda *a, **kw: SimpleNamespace(to_payload=lambda: {}),
    )

    def fake_jog(*a, **kw):
        captured.update(kw)
        return SimpleNamespace(accepted=True, completed=True, final_positions={}, message=None)

    monkeypatch.setattr(cli, "jog_linear_cli_units", fake_jog)
    args = build_parser().parse_args([
        "agent", "jog", "--frame", "world", "--x-mm", "5",
        "--speed-mm-s", "20", "--acceleration-mm-s2", "80",
    ])
    assert cli._cmd_agent_jog(args) == 0
    assert captured["speed_mm_s"] == pytest.approx(20)
    assert captured["acceleration_mm_s2"] == pytest.approx(80)


def test_agent_jog_trace_records_motion_hold_and_later_feedback(
    monkeypatch, tmp_path, capsys
) -> None:
    from soarm101_motion import SOARM101
    from soarm101_motion.cli import main as cli
    from soarm101_motion.types import MotionResult

    trace_file = tmp_path / "jog.jsonl"
    arm = SOARM101.simulated()
    monkeypatch.setattr(cli, "_arm_from_args", lambda *a, **kw: arm)
    monkeypatch.setattr(cli, "_agent_require_authority", lambda *a: {"armed": True})
    monkeypatch.setattr(cli, "_agent_calibration_id", lambda *a, **kw: "sha256:motor")
    monkeypatch.setattr(
        cli, "WorkspaceCalibrationStore",
        lambda *a: SimpleNamespace(load=lambda: object()),
    )
    monkeypatch.setattr(
        cli, "evaluate_agent_jog",
        lambda *a, **kw: SimpleNamespace(to_payload=lambda: {"maximum_distance_mm": 10}),
    )
    move_calls = []

    def fake_jog(arm, **kwargs):
        move_calls.append(kwargs)
        joints = dict(arm.get_joint_positions().positions)
        arm.backend.write_joint_positions(joints)
        return MotionResult(True, True, final_positions=joints)

    monkeypatch.setattr(cli, "jog_linear_cli_units", fake_jog)
    args = build_parser().parse_args([
        "agent", "jog", "--x-mm", "2", "--trace-file", str(trace_file),
    ])
    assert cli._cmd_agent_jog(args) == 0
    output = json.loads(capsys.readouterr().out)
    assert output["trace_file"] == str(trace_file)
    assert output["trace_summary"]["feedback_count"] >= 2
    assert len(move_calls) == 1

    events = [json.loads(line) for line in trace_file.read_text().splitlines()]
    markers = [e["marker"] for e in events if e["event"] == "marker"]
    assert markers == [
        "preflight", "motion_start", "motion_completed_before_hold",
        "hold_start", "hold_complete", "post_hold_immediate", "post_hold_2s",
    ]
    assert events[0]["action"] == "agent_jog"
    assert "calibration_id" in events[0]  # trace uses runtime identity, not a supplied ID
    assert events[0]["delta_model_mm"] == [2.0, 0.0, 0.0]
    assert any(e["event"] == "command" for e in events)
    for name in ("motion_completed_before_hold", "post_hold_immediate", "post_hold_2s"):
        event = next(e for e in events if e.get("marker") == name)
        assert set(event["joints_rad"]) == {
            "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll",
        }
        assert len(event["tcp_xyz_mm"]) == 3
    assert events[-1]["event"] == "trace_end"
    assert events[-1]["error"] is None


def test_agent_jog_trace_persists_execution_failure(monkeypatch, tmp_path) -> None:
    from soarm101_motion import SOARM101
    from soarm101_motion.cli import main as cli

    trace_file = tmp_path / "failed-jog.jsonl"
    arm = SOARM101.simulated()
    monkeypatch.setattr(cli, "_arm_from_args", lambda *a, **kw: arm)
    monkeypatch.setattr(cli, "_agent_require_authority", lambda *a: {"armed": True})
    monkeypatch.setattr(cli, "_agent_calibration_id", lambda *a, **kw: "sha256:motor")
    monkeypatch.setattr(
        cli, "WorkspaceCalibrationStore",
        lambda *a: SimpleNamespace(load=lambda: object()),
    )
    monkeypatch.setattr(
        cli, "evaluate_agent_jog",
        lambda *a, **kw: SimpleNamespace(to_payload=lambda: {}),
    )
    def fail_jog(*a, **kwargs):
        raise RuntimeError("test: simulated motor fault")

    monkeypatch.setattr(cli, "jog_linear_cli_units", fail_jog)
    args = build_parser().parse_args([
        "agent", "jog", "--x-mm", "2", "--trace-file", str(trace_file),
    ])
    with pytest.raises(RuntimeError, match="simulated motor fault"):
        cli._cmd_agent_jog(args)
    events = [json.loads(line) for line in trace_file.read_text().splitlines()]
    assert any(e.get("marker") == "motion_start" for e in events)
    assert not any(e.get("marker") == "hold_complete" for e in events)
    assert "simulated motor fault" in events[-1]["error"]


def test_agent_authority_is_time_bounded_and_identity_bound(tmp_path) -> None:
    store = AgentAuthorityStore(tmp_path / "authority.json")
    authority = store.issue(
        robot_id="so101",
        calibration_id="sha256:motor",
        minutes=2.0,
        now=100.0,
    )

    assert authority.active(now=219.0)
    assert not authority.active(now=220.0)
    assert store.require(
        robot_id="so101",
        calibration_id="sha256:motor",
        now=150.0,
    ).robot_id == "so101"

    with pytest.raises(PermissionError, match="robot"):
        store.require(
            robot_id="other",
            calibration_id="sha256:motor",
            now=150.0,
        )
    with pytest.raises(PermissionError, match="calibration"):
        store.require(
            robot_id="so101",
            calibration_id="sha256:changed",
            now=150.0,
        )
    with pytest.raises(PermissionError, match="expired"):
        store.require(
            robot_id="so101",
            calibration_id="sha256:motor",
            now=221.0,
        )
    assert not store.path.exists()


def test_agent_jog_allows_50_mm_only_above_100_mm() -> None:
    workspace = _identity_workspace()
    decision = evaluate_agent_jog(
        workspace,
        active_calibration_id="sha256:motor",
        current_model_position_m=(0.1, 0.1, 0.101),
        delta_model_m=(0.03, 0.04, 0.0),
    )
    assert decision.maximum_distance_m == pytest.approx(0.050)
    assert decision.requested_distance_m == pytest.approx(0.050)

    with pytest.raises(PermissionError, match="50.0 mm limit"):
        evaluate_agent_jog(
            workspace,
            active_calibration_id="sha256:motor",
            current_model_position_m=(0.1, 0.1, 0.101),
            delta_model_m=(0.051, 0.0, 0.0),
        )


def test_agent_jog_tightens_to_10_mm_near_ground() -> None:
    workspace = _identity_workspace()
    decision = evaluate_agent_jog(
        workspace,
        active_calibration_id="sha256:motor",
        current_model_position_m=(0.1, 0.1, 0.080),
        delta_model_m=(0.006, 0.008, 0.0),
    )
    assert decision.maximum_distance_m == pytest.approx(0.010)
    assert decision.requested_distance_m == pytest.approx(0.010)

    with pytest.raises(PermissionError, match="10.0 mm limit"):
        evaluate_agent_jog(
            workspace,
            active_calibration_id="sha256:motor",
            current_model_position_m=(0.1, 0.1, 0.080),
            delta_model_m=(0.011, 0.0, 0.0),
        )


def test_agent_jog_rejects_entering_calibrated_ground_margin() -> None:
    workspace = _identity_workspace()
    with pytest.raises(PermissionError, match="10.0 mm calibrated ground-plane safety margin"):
        evaluate_agent_jog(
            workspace,
            active_calibration_id="sha256:motor",
            current_model_position_m=(0.1, 0.1, 0.015),
            delta_model_m=(0.0, 0.0, -0.006),
        )


def test_agent_jog_requires_matching_workspace_calibration() -> None:
    workspace = _identity_workspace()
    with pytest.raises(PermissionError, match="does not match"):
        evaluate_agent_jog(
            workspace,
            active_calibration_id="sha256:other",
            current_model_position_m=np.array([0.1, 0.1, 0.1]),
            delta_model_m=np.array([0.001, 0.0, 0.0]),
        )
