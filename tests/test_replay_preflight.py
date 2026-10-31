from dataclasses import replace

import pytest

from soarm101_motion.asset_catalog import discover_assets
from soarm101_motion.exceptions import CalibrationError
from soarm101_motion.replay_policy import ReplayPolicy
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.replay_preflight import preflight_recording, preflight_sequence
from soarm101_motion.sequences import MotionSequence, SequenceStep
from test_asset_catalog import libraries


def setup_plan(tmp_path, steps=None):
    poses, trajectories, sequences = libraries(tmp_path)
    sequences.save(MotionSequence("seq", tuple(steps or (
        SequenceStep("point", {"name": "pose one"}),
        SequenceStep("trajectory", {"name": "wave", "kind": "raw", "loops": 2}),
        SequenceStep("wait", {"seconds": 1}),
    )), metadata={"source_robot_id": "arm", "source_calibration_id": "cal-1"}))
    catalog = discover_assets(poses, trajectories, sequences)
    policy = ReplayPolicy.from_document({
        "schema_version": 1, "robot_id": "arm", "calibration_id": "cal-1",
        "allowed_assets": {item.id: item.revision for item in catalog},
        "max_speed_scale": 0.5, "max_steps": 10, "max_duration_s": 30,
        "max_start_error_rad": 0.05,
        "max_start_gripper_error": 0.05,
    })
    return policy, poses, trajectories, sequences


def test_preflight_pins_every_dependency_and_records_entry(tmp_path):
    policy, poses, trajectories, sequences = setup_plan(tmp_path)
    report = preflight_sequence(policy, "soarm101/sequence/seq", poses, trajectories,
                                sequences, speed_scale=0.5)
    assert {item[0] for item in report.dependencies} == {
        "soarm101/sequence/seq", "soarm101/pose/pose%20one", "soarm101/recording/raw/wave"
    }
    assert report.expanded_steps == 4
    assert report.known_duration_s == 5
    assert report.entries[0].occurrences == 2
    assert len(report.entries[0].joints_rad) == 5
    assert len(report.pose_targets) == 1
    assert report.pose_targets[0].asset_id == "soarm101/pose/pose%20one"
    assert len(report.pose_targets[0].joints_rad) == 5
    assert report.pose_targets[0].gripper is None
    assert [(step.index, step.kind) for step in report.planned_steps] == [
        (0, "point"), (1, "trajectory"), (2, "wait")
    ]
    assert report.planned_steps[0].revision == dict(report.dependencies)[
        "soarm101/pose/pose%20one"
    ]
    assert report.planned_steps[1].revision == dict(report.dependencies)[
        "soarm101/recording/raw/wave"
    ]
    assert report.planned_steps[1].entry == report.entries[0]
    assert report.planned_steps[1].speed_scale == 0.5
    assert report.planned_steps[2].wait_seconds == 1


@pytest.mark.parametrize(("kind", "gripper_expected"), [("point", None), ("home", 0.5), ("rest", 0.5)])
def test_preflight_materializes_each_pose_step_target(tmp_path, kind, gripper_expected):
    policy, poses, trajectories, sequences = setup_plan(
        tmp_path, [SequenceStep(kind, {"name": "pose one"} if kind == "point" else {})]
    )
    if kind in {"home", "rest"}:
        poses.save(kind, poses.require("pose one"))
        catalog = discover_assets(poses, trajectories, sequences)
        policy = replace(policy, allowed_assets=tuple((item.id, item.revision) for item in catalog))
    report = preflight_sequence(policy, "soarm101/sequence/seq", poses, trajectories,
                                sequences, speed_scale=0.5)
    assert len(report.pose_targets) == 1
    expected_name = "pose%20one" if kind == "point" else kind
    assert report.pose_targets[0].asset_id == f"soarm101/pose/{expected_name}"
    assert report.pose_targets[0].joints_rad == tuple((joint, 0.0) for joint in ARM_JOINTS)
    assert report.pose_targets[0].gripper == gripper_expected


def test_rejects_unpinned_changed_and_mismatched_dependencies(tmp_path):
    policy, poses, trajectories, sequences = setup_plan(tmp_path)
    incomplete = replace(policy, allowed_assets=tuple(
        pair for pair in policy.allowed_assets if "/pose/" not in pair[0]
    ))
    with pytest.raises(PermissionError, match="not allowed"):
        preflight_sequence(incomplete, "soarm101/sequence/seq", poses, trajectories,
                           sequences, speed_scale=0.5)
    poses.save("pose one", replace(poses.require("pose one"), gripper=0.2))
    with pytest.raises(PermissionError, match="revision"):
        preflight_sequence(policy, "soarm101/sequence/seq", poses, trajectories,
                           sequences, speed_scale=0.5)
    mismatched = replace(policy, calibration_id="other")
    with pytest.raises(CalibrationError, match="calibration"):
        preflight_sequence(mismatched, "soarm101/sequence/seq", poses, trajectories,
                           sequences, speed_scale=0.5)


@pytest.mark.parametrize("step", [
    SequenceStep("primitive", {"name": "unbounded"}),
    SequenceStep("point", {"name": "pose one", "mode": "linear"}),
    SequenceStep("trajectory", {"name": "wave", "kind": "raw", "loops": 100}),
    SequenceStep("wait", {"seconds": 31}),
    SequenceStep("gripper", {"position": 2}),
    SequenceStep("point", {"name": "pose one", "joint_overrides_deg": {"elbow_flex": 10}}),
])
def test_rejects_unsupported_or_unbounded_steps_before_execution(tmp_path, step):
    policy, poses, trajectories, sequences = setup_plan(tmp_path, [step])
    with pytest.raises(ValueError):
        preflight_sequence(policy, "soarm101/sequence/seq", poses, trajectories,
                           sequences, speed_scale=0.5)


def test_detects_mutation_during_preflight(tmp_path, monkeypatch):
    policy, poses, trajectories, sequences = setup_plan(tmp_path)
    from soarm101_motion import replay_preflight
    original = replay_preflight.load_pinned_recording

    def mutate_after_snapshot(*args):
        result = original(*args)
        entry = trajectories.entries()[0]
        entry.metadata_path.write_text(entry.metadata_path.read_text() + " ")
        return result

    monkeypatch.setattr(replay_preflight, "load_pinned_recording", mutate_after_snapshot)
    with pytest.raises(RuntimeError, match="changed during preflight"):
        preflight_sequence(policy, "soarm101/sequence/seq", poses, trajectories,
                           sequences, speed_scale=0.5)


def test_recording_preflight_checks_pinned_entry_and_duration(tmp_path):
    policy, poses, trajectories, sequences = setup_plan(tmp_path)
    joints = dict.fromkeys(ARM_JOINTS, 0.0)
    args = (policy, "soarm101/recording/raw/wave", poses, trajectories, sequences)
    report = preflight_recording(*args, speed_scale=0.5, actual_joints=joints, actual_gripper=0.5)
    assert report.known_duration_s == 2
    assert report.entry.gripper == 0.5
    assert report.revision == dict(policy.allowed_assets)[report.asset_id]
    with pytest.raises(PermissionError, match="gripper"):
        preflight_recording(*args, speed_scale=0.5, actual_joints=joints, actual_gripper=0.7)
    with pytest.raises(PermissionError, match="start state"):
        preflight_recording(*args, speed_scale=0.5,
                            actual_joints={**joints, ARM_JOINTS[0]: 0.2}, actual_gripper=0.5)
    with pytest.raises(ValueError, match="duration"):
        preflight_recording(*args, speed_scale=0.01, actual_joints=joints, actual_gripper=0.5)


def test_recording_rejects_mutation_and_calibration_mismatch(tmp_path):
    policy, poses, trajectories, sequences = setup_plan(tmp_path)
    joints = dict.fromkeys(ARM_JOINTS, 0.0)
    args = ("soarm101/recording/raw/wave", poses, trajectories, sequences)
    with pytest.raises(PermissionError, match="calibration"):
        preflight_recording(replace(policy, calibration_id="other"), *args, speed_scale=0.5,
                            actual_joints=joints, actual_gripper=0.5)
    entry = trajectories.entries()[0]
    entry.metadata_path.write_text(entry.metadata_path.read_text() + " ")
    with pytest.raises(PermissionError, match="revision"):
        preflight_recording(policy, *args, speed_scale=0.5,
                            actual_joints=joints, actual_gripper=0.5)
