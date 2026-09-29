from __future__ import annotations

import json

import pytest

from soarm101_motion.camera import CameraSettings, CameraSettingsStore
from soarm101_motion.workstation import (
    ArmConnectionProfile,
    WorkstationProfile,
    WorkstationProfileStore,
)


def test_workstation_profile_round_trip(tmp_path) -> None:
    path = tmp_path / "workstation.json"
    store = WorkstationProfileStore(path, legacy_camera_path=tmp_path / "legacy.json")
    profile = WorkstationProfile(
        follower=ArmConnectionProfile(
            port="/dev/ttyACM0",
            robot_id="so101",
            calibration=str(tmp_path / "follower.json"),
        ),
        leader=ArmConnectionProfile(
            port="/dev/ttyACM1",
            robot_id="so101-leader",
            calibration=str(tmp_path / "leader.json"),
        ),
        cameras={
            "overhead": CameraSettings(device="/dev/video2"),
            "wrist": CameraSettings(device="/dev/video4", width=640, height=480),
        },
        selected_camera="wrist",
    ).validated()

    assert store.save(profile) == profile
    loaded = store.load()

    assert loaded.follower.port == "/dev/ttyACM0"
    assert loaded.leader.port == "/dev/ttyACM1"
    assert loaded.selected_camera == "wrist"
    assert loaded.camera("overhead").device == "/dev/video2"
    assert loaded.camera("wrist").width == 640

    payload = json.loads(path.read_text(encoding="utf-8"))
    assert sorted(payload["cameras"]) == ["overhead", "wrist"]


def test_workstation_migrates_legacy_camera_settings(tmp_path) -> None:
    legacy = tmp_path / "camera.json"
    CameraSettingsStore(legacy).save(
        CameraSettings(device="/dev/video7", mirror=True)
    )
    store = WorkstationProfileStore(
        tmp_path / "workstation.json",
        legacy_camera_path=legacy,
    )

    profile = store.load()

    assert profile.selected_camera == "camera"
    assert profile.camera("camera").device == "/dev/video7"
    assert profile.camera("camera").mirror is True


def test_workstation_rejects_duplicate_camera_devices() -> None:
    with pytest.raises(ValueError, match="same device"):
        WorkstationProfile(
            cameras={
                "overhead": CameraSettings(device="/dev/video2"),
                "wrist": CameraSettings(device="/dev/video2"),
            }
        ).validated()


def test_workstation_camera_rename_preserves_selection() -> None:
    profile = WorkstationProfile(
        cameras={"camera": CameraSettings(device="/dev/video2")},
        selected_camera="camera",
    ).validated()

    renamed = profile.renamed_camera("camera", "overhead")

    assert renamed.selected_camera == "overhead"
    assert "camera" not in renamed.cameras
    assert renamed.camera("overhead").device == "/dev/video2"
