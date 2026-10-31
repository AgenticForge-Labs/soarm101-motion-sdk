from dataclasses import replace

import pytest

from soarm101_motion.replay_snapshot import load_pinned_recording
from test_replay_preflight import setup_plan


def test_snapshot_is_content_bound_and_survives_later_file_mutation(tmp_path):
    policy, _, library, _ = setup_plan(tmp_path)
    ident = "soarm101/recording/raw/wave"
    snapshot = load_pinned_recording(policy, ident, library)
    first = snapshot.trajectory()
    entry = library.entries()[0]
    entry.metadata_path.write_text(entry.metadata_path.read_text() + " ")
    assert snapshot.trajectory().joints_rad.tolist() == first.joints_rad.tolist()
    with pytest.raises(PermissionError, match="revision"):
        load_pinned_recording(policy, ident, library)
    with pytest.raises(ValueError, match="content"):
        replace(snapshot, metadata_bytes=snapshot.metadata_bytes + b" ").trajectory()


def test_snapshot_rejects_symlinks_and_unapproved_ids(tmp_path):
    policy, _, library, _ = setup_plan(tmp_path)
    ident = "soarm101/recording/raw/wave"
    with pytest.raises(ValueError, match="missing"):
        load_pinned_recording(policy, "soarm101/recording/raw/other", library)
    entry = library.entries()[0]
    original = entry.metadata_path.read_bytes()
    other = tmp_path / "outside.json"
    other.write_bytes(original)
    entry.metadata_path.unlink()
    entry.metadata_path.symlink_to(other)
    with pytest.raises(ValueError, match="non-symlink"):
        load_pinned_recording(policy, ident, library)
