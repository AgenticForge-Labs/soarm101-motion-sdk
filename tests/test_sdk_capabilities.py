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

    def stop(self):
        self.calls.append(("stop", None))
        return "holding"


def test_registry_inspection_is_pure_and_exposes_units() -> None:
    specs = SDK_CAPABILITIES.describe()
    names = [spec["name"] for spec in specs]
    assert len(names) == len(set(names))
    assert {"read_pose", "solve_ik", "move_joints", "jog_joint", "move_linear",
            "jog_cartesian", "move_gripper", "stop"} == set(names)
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
