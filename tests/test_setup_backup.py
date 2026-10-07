import json

import pytest

from soarm101_motion.calibration import (
    MotorCalibration,
    SO101Calibration,
    save_versioned_calibration,
)
from soarm101_motion.constants import ALL_MOTORS, MOTOR_IDS
from soarm101_motion.setup_backup import export_setup, restore_setup, inspect_backup


def calibration(offset=100):
    return SO101Calibration(
        motors={name: MotorCalibration(MOTOR_IDS[name], 0, 0, offset, 3900) for name in ALL_MOTORS}
    )


def make_setup(root):
    save_versioned_calibration(
        calibration(),
        robot_id="so101",
        current_path=root / "calibration/so101.json",
        history_dir=root / "calibration/history/so101",
    )


def test_backup_roundtrip_preserves_history_and_previous_files(tmp_path):
    source, target = tmp_path / "source", tmp_path / "target"
    make_setup(source)
    backup = tmp_path / "backup.json"
    assert export_setup(backup, root=source) == 2
    old = target / "calibration/so101.json"
    calibration(200).save(old)
    previous = old.read_bytes()
    recovery = restore_setup(backup, root=target)
    assert SO101Calibration.load(old).fingerprint == calibration().fingerprint
    assert (recovery / "calibration/so101.json").read_bytes() == previous
    assert len(list((target / "calibration/history/so101").glob("*.json"))) == 1


@pytest.mark.parametrize(
    "name",
    ["../escape.json", "/tmp/escape.json", "calibration/../../escape.json", "arbitrary.json"],
)
def test_restore_rejects_paths_before_writing(tmp_path, name):
    source = tmp_path / "bad.json"
    source.write_text(json.dumps({"format": "soarm101-setup", "version": 1, "files": {name: {}}}))
    target = tmp_path / "target"
    with pytest.raises(ValueError):
        restore_setup(source, root=target)
    assert not target.exists()


def test_corrupt_calibration_rejects_whole_restore(tmp_path):
    source = tmp_path / "source"
    make_setup(source)
    backup = tmp_path / "backup.json"
    export_setup(backup, root=source)
    data = json.loads(backup.read_text())
    data["files"]["calibration/so101.json"]["calibration_id"] = "wrong"
    backup.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="fingerprint"):
        restore_setup(backup, root=tmp_path / "target")
    assert not (tmp_path / "target").exists()


def test_restore_rolls_back_partial_install(tmp_path, monkeypatch):
    import soarm101_motion.setup_backup as storage

    source, target = tmp_path / "source", tmp_path / "target"
    make_setup(source)
    make_setup(target)
    calibration(200).save(target / "calibration/so101.json")
    for snapshot in (target / "calibration/history/so101").glob("*.json"):
        snapshot.unlink()
    original = (target / "calibration/so101.json").read_bytes()
    backup = tmp_path / "backup.json"
    export_setup(backup, root=source)
    real_write = storage._atomic_write
    failed = False

    def fail_once(path, data):
        nonlocal failed
        if path.parent == target / "calibration/history/so101" and not failed:
            failed = True
            raise OSError("disk failure")
        real_write(path, data)

    monkeypatch.setattr(storage, "_atomic_write", fail_once)
    with pytest.raises(OSError):
        restore_setup(backup, root=target)
    assert (target / "calibration/so101.json").read_bytes() == original


def test_restore_rebases_calibration_reference(tmp_path):
    source = tmp_path / "source"
    make_setup(source)
    (source / "workstation.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "follower": {"robot_id": "so101", "calibration": "/old/computer/calibration.json"},
            }
        )
    )
    backup = tmp_path / "backup.json"
    export_setup(backup, root=source)
    assert len(inspect_backup(backup)) == 3
    target = tmp_path / "target"
    restore_setup(backup, root=target)
    restored = json.loads((target / "workstation.json").read_text())
    assert restored["follower"]["calibration"] == str(target / "calibration/so101.json")


def test_backup_refuses_repository_destination(tmp_path):
    source = tmp_path / "source"
    make_setup(source)
    repo = tmp_path / "repo"
    (repo / ".git").mkdir(parents=True)
    with pytest.raises(ValueError, match="Git repository"):
        export_setup(repo / "setup.json", root=source)


def test_restore_rejects_symlink_destination(tmp_path):
    source, target = tmp_path / "source", tmp_path / "target"
    make_setup(source)
    backup = tmp_path / "backup.json"
    export_setup(backup, root=source)
    target.mkdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    (target / "calibration").symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError, match="escapes"):
        restore_setup(backup, root=target)
    assert not list(outside.iterdir())
