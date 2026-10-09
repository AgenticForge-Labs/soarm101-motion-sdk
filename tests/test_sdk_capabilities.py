"""Hardware-free contract tests for the shared typed SDK capability surface."""

from __future__ import annotations

import json
from types import SimpleNamespace

import pytest

from soarm101_motion.cli.main import main
from soarm101_motion.sdk_capabilities import SDK_CAPABILITIES


class FakeArm:
    def __init__(self) -> None:
        self.calls: list[tuple[str, object]] = []
        self.tool = SimpleNamespace(move=self.gripper_move)

    def get_joint_positions(self):
        self.calls.append(("joint_read", None))
        return SimpleNamespace(positions={"shoulder_pan": 0.1})

    def get_position(self):
        self.calls.append(("tcp_read", None))
        return SimpleNamespace(xyz_rpy=lambda: (0.1, 0.2, 0.3, 0.0, 0.0, 0.0))

    def move_joints(self, positions, *, speed, acceleration, relative=False):
        event = "joint_jog" if relative else "joints"
        self.calls.append((event, (positions, speed, acceleration)))
        return "joints complete"

    def move_linear(self, target, **kwargs):
        self.calls.append(("linear", (target, kwargs)))
        return "linear complete"

    def solve_ik(self, target, *, orientation_mode):
        self.calls.append(("ik", orientation_mode))
        return "solved"

    def gripper_move(self, position):
        self.calls.append(("gripper", position))
        return "gripper complete"

    def get_state(self):
        self.calls.append(("hardware_state", None))
        return SimpleNamespace(connected=True, moving=False)

    def get_effort_safety_status(self, *, refresh=False):
        self.calls.append(("effort_status", refresh))
        return {"enabled": False, "refreshed": refresh}

    def stop(self):
        self.calls.append(("stop", None))
        return "holding"


def test_registry_inspection_is_pure_and_exposes_units() -> None:
    specs = SDK_CAPABILITIES.describe()
    names = [spec["name"] for spec in specs]
    assert len(names) == len(set(names))
    assert {"read_pose", "solve_ik", "move_joints", "jog_joint", "move_linear",
            "jog_cartesian", "move_gripper", "list_saved_poses",
            "capture_saved_pose", "validate_saved_pose", "replay_saved_pose",
            "capture_camera", "camera_profiles", "read_effort_status",
            "read_hardware_state", "stop"} == set(names)
    joint = SDK_CAPABILITIES.get("move_joints").describe()
    assert joint["effect"] == "motion"
    assert joint["agent_eligible"] is False
    assert joint["arguments"][0]["unit"] == "rad"
    assert SDK_CAPABILITIES.get("stop").agent_eligible is True
    assert SDK_CAPABILITIES.get("read_pose").effect == "read"


def test_registry_checks_payload_before_sdk_side_effect() -> None:
    arm = FakeArm()
    bad = [
        ("move_joints", {"positions_rad": [0.0] * 5, "raw_register_write": 6}),
        ("move_joints", {"positions_rad": [0.0] * 4}),
        ("move_joints", {"positions_rad": [0.0, 0.0, 0.0, 0.0, float("nan")]}),
        ("move_joints", {"positions_rad": [0.0] * 5, "speed_rad_s": "fast"}),
        ("move_joints", {"positions_rad": [0.0] * 5, "speed_rad_s": True}),
        ("move_linear", {"target_xyz_mm": [1.0, 2.0, 3.0],
                         "target_rpy_deg": [0.0, 0.0, 0.0], "orientation_mode": "unsafe"}),
        ("move_gripper", {"position": 1.1}),
        ("not-an-action", {}),
    ]
    for name, payload in bad:
        with pytest.raises(ValueError):
            SDK_CAPABILITIES.dispatch(name, arm, payload)
    assert arm.calls == []


def test_registry_keeps_sdk_joint_dynamics_and_stop_semantics() -> None:
    arm = FakeArm()
    base = {"positions_rad": [0.1] * 5}
    assert SDK_CAPABILITIES.dispatch("move_joints", arm, base) == "joints complete"
    assert arm.calls[-1] == ("joints", ((0.1,) * 5, None, None))
    explicit = {**base, "speed_rad_s": 0.0, "acceleration_rad_s2": 2.0}
    SDK_CAPABILITIES.dispatch("move_joints", arm, explicit)
    assert arm.calls[-1] == ("joints", ((0.1,) * 5, 0.0, 2.0))
    # Registry delegates even a zero speed to SDK rejection; never treats it
    # as a request to remove the SDK speed guard.
    assert SDK_CAPABILITIES.dispatch("stop", arm, {}) == "holding"
    assert arm.calls[-1] == ("stop", None)


def test_registry_read_pose_and_gripper_use_existing_sdk_calls() -> None:
    arm = FakeArm()
    data = SDK_CAPABILITIES.dispatch("read_pose", arm, {})
    assert data["tcp_xyz_mm"] == pytest.approx([100.0, 200.0, 300.0])
    assert data["joint_positions_rad"]["shoulder_pan"] == pytest.approx(0.1)
    assert SDK_CAPABILITIES.dispatch("move_gripper", arm, {"position": 0.5}) == "gripper complete"
    assert arm.calls[-1] == ("gripper", 0.5)


def test_cli_exposes_registry_metadata_without_hardware(capsys) -> None:
    assert main(["sdk-capabilities", "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["schema_version"] == 1
    assert any(action["name"] == "jog_cartesian" for action in payload["actions"])
    assert any(action["name"] == "move_linear" and action["effect"] == "motion"
               for action in payload["actions"])


def test_existing_read_cli_json_parity_in_simulation(capsys) -> None:
    assert main(["read", "--simulation", "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert list(payload) == ["joint_positions_rad", "tcp_xyz_mm", "tcp_rpy_deg"]
    assert len(payload["tcp_xyz_mm"]) == 3
    assert len(payload["tcp_rpy_deg"]) == 3


def test_single_joint_jog_uses_relative_guarded_sdk_and_validates_names() -> None:
    arm = FakeArm()
    request = {
        "joint": "elbow_flex", "delta_rad": 0.03,
        "speed_rad_s": 0.1, "acceleration_rad_s2": 0.3,
    }
    assert SDK_CAPABILITIES.dispatch("jog_joint", arm, request) == "joints complete"
    assert arm.calls[-1] == ("joint_jog", ({"elbow_flex": 0.03}, 0.1, 0.3))
    before = list(arm.calls)
    with pytest.raises(ValueError, match="joint"):
        SDK_CAPABILITIES.dispatch("jog_joint", arm, {**request, "joint": "servo_6"})
    assert arm.calls == before


def test_cli_single_joint_operator_jog_needs_explicit_confirmation(capsys) -> None:
    assert main(["jog-joint", "shoulder_pan", "--delta-deg", "2", "--simulation"]) == 2
    assert "Refusing to move without --yes" in capsys.readouterr().err


def test_registry_shared_effort_and_state_reads() -> None:
    arm = FakeArm()
    assert SDK_CAPABILITIES.dispatch(
        "read_effort_status", arm, {"refresh": True},
    ) == {"enabled": False, "refreshed": True}
    assert arm.calls[-1] == ("effort_status", True)
    state = SDK_CAPABILITIES.dispatch("read_hardware_state", arm, {})
    assert state.connected and not state.moving
    assert arm.calls[-1] == ("hardware_state", None)
    before = list(arm.calls)
    with pytest.raises(ValueError, match="boolean"):
        SDK_CAPABILITIES.dispatch("read_effort_status", arm, {"refresh": "yes"})
    assert arm.calls == before
    with pytest.raises(ValueError, match="connected SDK session"):
        SDK_CAPABILITIES.dispatch("read_hardware_state", None, {})


def test_registry_lists_saved_poses_without_hardware(tmp_path, monkeypatch) -> None:
    import soarm101_motion.poses as poses
    from soarm101_motion.constants import ARM_JOINTS

    monkeypatch.setattr(
        poses, "default_pose_library_path",
        lambda robot_id: tmp_path / f"{robot_id}.json",
    )
    library = poses.PoseLibrary("so101")
    library.save(
        "test_point",
        poses.SavedPose(
            joints={name: 0.0 for name in ARM_JOINTS},
            gripper=0.5,
            tcp_xyz_rpy=(0.0, 0.0, 0.2, 0.0, 0.0, 0.0),
            source="follower",
            created_at="2026-10-09T00:00:00Z",
        ),
    )
    rows = SDK_CAPABILITIES.dispatch(
        "list_saved_poses", None, {"robot_id": "so101"},
    )
    assert rows == [{
        "name": "test_point",
        "source": "follower",
        "created_at": "2026-10-09T00:00:00Z",
    }]
    assert main(["pose", "list", "--robot-id", "so101"]) == 0
    with pytest.raises(ValueError, match="nonempty"):
        SDK_CAPABILITIES.dispatch("list_saved_poses", None, {"robot_id": " "})


def test_registry_pose_capture_and_replay_are_calibration_bound(
    monkeypatch, tmp_path,
) -> None:
    from soarm101_motion import SOARM101
    import soarm101_motion.poses as poses

    monkeypatch.setattr(
        poses, "default_pose_library_path",
        lambda robot_id: tmp_path / f"{robot_id}.json",
    )
    with SOARM101.simulated() as arm:
        path = SDK_CAPABILITIES.dispatch(
            "capture_saved_pose", arm,
            {"robot_id": arm.config.robot_id, "name": "test", "source": "follower"},
        )
        assert path.exists()
        bound = SDK_CAPABILITIES.dispatch(
            "validate_saved_pose", arm,
            {"robot_id": arm.config.robot_id, "name": "test"},
        )
        assert bound.source == "follower"
        # Registry never enables torque: the owner of the session must do it.
        assert not arm.get_state().torque_enabled
        arm.enable()
        arm_result, gripper_result = SDK_CAPABILITIES.dispatch(
            "replay_saved_pose", arm,
            {"robot_id": arm.config.robot_id, "name": "test", "mode": "joint"},
        )
        assert arm_result.completed and gripper_result.completed


def test_registry_pose_replay_rejects_provenance_before_motion(tmp_path, monkeypatch) -> None:
    import soarm101_motion.poses as poses
    from soarm101_motion.constants import ARM_JOINTS

    monkeypatch.setattr(
        poses, "default_pose_library_path",
        lambda robot_id: tmp_path / f"{robot_id}.json",
    )
    poses.PoseLibrary("so101").save(
        "bad",
        poses.SavedPose(
            joints={name: 0.0 for name in ARM_JOINTS},
            gripper=0.5,
            tcp_xyz_rpy=(0.0, 0.0, 0.2, 0.0, 0.0, 0.0),
            source_robot_id="so101",
            source_calibration_id="sha256:stale",
        ),
    )

    class RejectingArm(FakeArm):
        def require_artifact_calibration(self, provenance, *, artifact_label):
            self.calls.append(("provenance", provenance))
            raise ValueError("stale calibration")

    arm = RejectingArm()
    with pytest.raises(ValueError, match="stale calibration"):
        SDK_CAPABILITIES.dispatch(
            "replay_saved_pose", arm,
            {"robot_id": "so101", "name": "bad"},
        )
    assert [name for name, _ in arm.calls] == ["provenance"]


def test_registry_camera_capture_uses_trusted_settings_not_request_overrides(
    monkeypatch, tmp_path,
) -> None:
    from soarm101_motion.camera import CameraSettings
    import soarm101_motion.camera as camera

    captured = []

    class FakeCapture:
        def __init__(self, settings):
            captured.append(settings)

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return None

        def snapshot(self, output):
            captured.append(output)
            return tmp_path / "capture.jpg", {
                "path": str(tmp_path / "capture.jpg"),
                "width": 640,
                "height": 480,
                "device": "/dev/video9",
            }

    monkeypatch.setattr(camera, "CameraCapture", FakeCapture)
    settings = CameraSettings(device="/dev/video9")
    payload = SDK_CAPABILITIES.dispatch(
        "capture_camera", None,
        {"name": "overhead", "output": str(tmp_path / "capture.jpg")},
        camera_settings=settings,
    )
    assert payload["name"] == "overhead"
    assert payload["path"] == str(tmp_path / "capture.jpg")
    assert captured == [settings, str(tmp_path / "capture.jpg")]
    with pytest.raises(ValueError, match="unknown"):
        SDK_CAPABILITIES.dispatch(
            "capture_camera", None,
            {"name": "overhead", "device": "/dev/video0"},
            camera_settings=settings,
        )
    assert len(captured) == 2


def test_operator_camera_capture_keeps_cli_profile_and_output_parity(
    monkeypatch, tmp_path, capsys,
) -> None:
    import soarm101_motion.camera as camera
    from soarm101_motion.workstation import WorkstationProfileStore

    monkeypatch.setenv(
        "SOARM101_WORKSTATION_CONFIG", str(tmp_path / "workstation.json"),
    )
    records = []

    class FakeCapture:
        def __init__(self, settings):
            self.settings = settings

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return None

        def snapshot(self, output):
            records.append((self.settings.device, output))
            return tmp_path / "shot.jpg", {
                "path": str(tmp_path / "shot.jpg"), "device": self.settings.device,
                "width": 640, "height": 480,
            }

    monkeypatch.setattr(camera, "CameraCapture", FakeCapture)
    store = WorkstationProfileStore()
    # Set up the same persisted camera name that the CLI and GUI share.
    from soarm101_motion.camera import CameraSettings
    store.save(store.load().with_camera(
        "overhead", CameraSettings(device="/dev/video7"), select=True,
    ))
    output = str(tmp_path / "override.jpg")
    assert main(["camera", "capture", "--name", "overhead",
                 "--output", output, "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["name"] == "overhead"
    assert payload["device"] == "/dev/video7"
    assert records == [("/dev/video7", output)]
    assert main(["camera", "capture", "--all", "--json"]) == 0
    all_data = json.loads(capsys.readouterr().out)
    assert len(all_data["captures"]) == 1
    assert records[-1] == ("/dev/video7", None)


def test_registry_camera_profiles_use_persisted_source_without_devices(
    tmp_path, monkeypatch,
) -> None:
    from soarm101_motion.camera import CameraSettings
    from soarm101_motion.workstation import WorkstationProfileStore

    monkeypatch.setenv(
        "SOARM101_WORKSTATION_CONFIG", str(tmp_path / "workstation.json"),
    )
    store = WorkstationProfileStore()
    store.save(store.load().with_camera(
        "wrist", CameraSettings(device="/dev/video8"), select=True,
    ))
    all_profiles = SDK_CAPABILITIES.dispatch("camera_profiles", None, {})
    assert all_profiles["selected_camera"] == "wrist"
    assert all_profiles["cameras"]["wrist"]["device"] == "/dev/video8"
    named = SDK_CAPABILITIES.dispatch(
        "camera_profiles", None, {"name": "wrist"},
    )
    assert named["name"] == "wrist"
    assert named["device"] == "/dev/video8"
    with pytest.raises(KeyError):
        SDK_CAPABILITIES.dispatch("camera_profiles", None, {"name": "absent"})
