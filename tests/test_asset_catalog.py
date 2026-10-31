from dataclasses import asdict, replace
import json

import numpy as np
import pytest

from soarm101_motion.asset_catalog import discover_assets
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.poses import PoseLibrary, SavedPose
from soarm101_motion.sequences import MotionSequence, SequenceLibrary, SequenceStep
from soarm101_motion.trajectories import Trajectory, TrajectoryLibrary


def libraries(tmp_path):
    poses = PoseLibrary("arm", path=tmp_path / "poses.json")
    trajectories = TrajectoryLibrary("arm", root=tmp_path / "recordings")
    sequences = SequenceLibrary("arm", path=tmp_path / "sequences.json")
    pose = SavedPose(
        joints={name: 0 for name in ARM_JOINTS}, gripper=0.5, tcp_xyz_rpy=(0,) * 6,
        source_robot_id="arm", source_calibration_id="cal-1",
    )
    poses.save("pose one", pose)
    trajectories.save("wave", Trajectory(
        timestamps_s=np.array([0.0, 1.0]), joints_rad=np.zeros((2, len(ARM_JOINTS))),
        gripper=np.array([0.5, 0.6]),
        metadata={"source_robot_id": "arm", "source_calibration_id": "cal-1"},
    ), kind="raw")
    sequences.save(MotionSequence("seq", (SequenceStep("wait", {"seconds": 1}),)))
    return poses, trajectories, sequences


def test_discovery_is_deterministic_and_only_reports_calibration_match(tmp_path):
    libs = libraries(tmp_path)
    catalog = discover_assets(*libs, current_robot_id="arm", current_calibration_id="cal-1")
    assert [item.id for item in catalog] == [
        "soarm101/pose/pose%20one", "soarm101/recording/raw/wave", "soarm101/sequence/seq"
    ]
    assert [item.compatibility for item in catalog] == [
        "calibration_matched", "calibration_matched", "unchecked"
    ]
    assert catalog[1].duration_s == 1.0
    assert catalog == discover_assets(*libs, current_robot_id="arm", current_calibration_id="cal-1")
    json.dumps([asdict(item) for item in catalog], allow_nan=False)
    assert all(item.revision.startswith("sha256:") for item in catalog)


def test_mismatch_and_unknown_are_not_authorization(tmp_path):
    libs = libraries(tmp_path)
    mismatch = discover_assets(*libs, current_robot_id="arm", current_calibration_id="other")
    assert [item.compatibility for item in mismatch] == [
        "calibration_mismatch", "calibration_mismatch", "unchecked"
    ]
    assert all(item.compatibility == "unchecked" for item in discover_assets(*libs))


def test_revisions_change_when_content_changes(tmp_path):
    poses, trajectories, sequences = libraries(tmp_path)
    before = {item.id: item.revision for item in discover_assets(poses, trajectories, sequences)}
    pose = poses.require("pose one")
    poses.save("pose one", replace(pose, gripper=0.2))
    entry = trajectories.entries()[0]
    entry.metadata_path.write_text(entry.metadata_path.read_text() + " ")
    sequences.save(MotionSequence("seq", (SequenceStep("wait", {"seconds": 2}),)))
    after = {item.id: item.revision for item in discover_assets(poses, trajectories, sequences)}
    assert all(before[key] != after[key] for key in before)


def test_cross_robot_libraries_fail_closed(tmp_path):
    poses, trajectories, sequences = libraries(tmp_path)
    other = SequenceLibrary("other", path=tmp_path / "other.json")
    with pytest.raises(ValueError, match="same robot"):
        discover_assets(poses, trajectories, other)


def test_trajectory_change_during_discovery_is_rejected(tmp_path, monkeypatch):
    poses, trajectories, sequences = libraries(tmp_path)
    original = trajectories.load

    def load_and_modify(name, *, kind):
        loaded = original(name, kind=kind)
        entry = trajectories.entries()[0]
        entry.metadata_path.write_text(entry.metadata_path.read_text() + " ")
        return loaded

    monkeypatch.setattr(trajectories, "load", load_and_modify)
    with pytest.raises(RuntimeError, match="changed while catalog"):
        discover_assets(poses, trajectories, sequences)


def test_legacy_pose_revision_does_not_depend_on_generated_timestamp(tmp_path):
    poses, trajectories, sequences = libraries(tmp_path)
    data = json.loads(poses.path.read_text())
    del data["poses"]["pose one"]["created_at"]
    poses.path.write_text(json.dumps(data))
    first = discover_assets(poses, trajectories, sequences)[0].revision
    second = discover_assets(poses, trajectories, sequences)[0].revision
    assert first == second
