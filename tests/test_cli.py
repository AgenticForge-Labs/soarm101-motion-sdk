import json

import pytest

from soarm101_motion.cli.main import _arm_from_args, build_parser, main
from soarm101_motion.sequences import MotionSequence, SequenceLibrary, SequenceStep


def test_info(capsys) -> None:
    assert main(["info"]) == 0
    assert "soarm101-motion-sdk" in capsys.readouterr().out


def test_calibration_default_allows_two_full_sweeps() -> None:
    args = build_parser().parse_args(["calibrate", "--port", "/dev/ttyACM0"])
    assert args.seconds == 90.0


def test_sim_demo(capsys) -> None:
    assert main(["sim-demo"]) == 0
    assert "Final joints" in capsys.readouterr().out


def test_gui_pose_library_is_available_from_cli(tmp_path, monkeypatch, capsys) -> None:
    monkeypatch.setenv("HOME", str(tmp_path))
    common = ["--robot-id", "cli-simulation"]
    assert main(["pose", "capture", "home", *common, "--simulation"]) == 0
    assert main(["pose", "list", *common]) == 0
    assert "home" in capsys.readouterr().out
    assert main(["pose", "go", "home", *common, "--simulation", "--yes"]) == 0
    assert "completed=True" in capsys.readouterr().out


def test_gui_sequence_library_is_available_from_cli(tmp_path, monkeypatch, capsys) -> None:
    monkeypatch.setenv("HOME", str(tmp_path))
    library = SequenceLibrary("cli-simulation")
    library.save(MotionSequence("pause", (SequenceStep("wait", {"seconds": 0.01}),)))
    common = ["--robot-id", "cli-simulation"]
    assert main(["sequence", "list", *common]) == 0
    assert "pause" in capsys.readouterr().out
    assert main(["sequence", "run", "pause", *common, "--simulation", "--yes"]) == 0
    assert "completed=True" in capsys.readouterr().out


def test_discover_reports_role_without_motion(monkeypatch, capsys) -> None:
    monkeypatch.setattr(
        "soarm101_motion.cli.main.discover_so101_arms",
        lambda ports: [
            {
                "port": "/dev/ttyACM0",
                "status": "ok",
                "role": "follower",
                "voltage_v": 12.0,
                "motor_count": 6,
                "motor_total": 6,
                "error": None,
            }
        ],
    )
    assert main(["discover", "/dev/ttyACM0"]) == 0
    assert "follower (12.0 V, 6/6 motors)" in capsys.readouterr().out


def test_agent_facing_read_and_ik_json_in_simulation(capsys) -> None:
    assert main(["read", "--simulation", "--json"]) == 0
    state = json.loads(capsys.readouterr().out)
    assert set(state) == {"joint_positions_rad", "tcp_xyz_mm", "tcp_rpy_deg"}
    assert len(state["joint_positions_rad"]) == 5
    assert len(state["tcp_xyz_mm"]) == 3

    x_mm, y_mm, z_mm = state["tcp_xyz_mm"]
    assert (
        main(
            [
                "ik",
                "--simulation",
                "--x-mm",
                str(x_mm),
                "--y-mm",
                str(y_mm),
                "--z-mm",
                str(z_mm),
                "--orientation-mode",
                "position_only",
                "--json",
            ]
        )
        == 0
    )
    solution = json.loads(capsys.readouterr().out)
    assert solution["success"] is True
    assert solution["position_error_mm"] <= 0.5
    assert len(solution["joints_rad"]) == 5
    assert len(solution["joints_deg"]) == 5


def test_agent_facing_motion_commands_support_json_in_simulation(capsys) -> None:
    assert (
        main(
            [
                "jog",
                "--simulation",
                "--x-mm",
                "2",
                "--speed-mm-s",
                "10",
                "--acceleration-mm-s2",
                "40",
                "--json",
                "--yes",
            ]
        )
        == 0
    )
    jog = json.loads(capsys.readouterr().out)
    assert jog["accepted"] is True
    assert jog["completed"] is True

    assert main(["gripper", "--simulation", "0.5", "--json", "--yes"]) == 0
    gripper = json.loads(capsys.readouterr().out)
    assert gripper["accepted"] is True
    assert gripper["completed"] is True



def test_session_cli_defaults_to_saved_workstation_follower(tmp_path, monkeypatch) -> None:
    workstation = tmp_path / "workstation.json"
    calibration = tmp_path / "bench-follower.json"
    workstation.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "follower": {
                    "port": "/dev/ttyACM9",
                    "robot_id": "bench-follower",
                    "calibration": str(calibration),
                },
                "leader": {
                    "port": "/dev/ttyACM8",
                    "robot_id": "bench-leader",
                    "calibration": str(tmp_path / "bench-leader.json"),
                },
                "selected_camera": None,
                "cameras": {},
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("SOARM101_WORKSTATION_CONFIG", str(workstation))

    args = build_parser().parse_args(["read", "--json"])
    arm = _arm_from_args(args)

    assert arm.config.port == "/dev/ttyACM9"
    assert arm.config.robot_id == "bench-follower"
    assert arm.config.calibration_path == calibration


def test_explicit_session_port_overrides_saved_workstation_follower(
    tmp_path, monkeypatch
) -> None:
    workstation = tmp_path / "workstation.json"
    workstation.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "follower": {
                    "port": "/dev/ttyACM9",
                    "robot_id": "bench-follower",
                    "calibration": str(tmp_path / "bench-follower.json"),
                },
                "leader": {
                    "port": "",
                    "robot_id": "bench-leader",
                    "calibration": str(tmp_path / "bench-leader.json"),
                },
                "selected_camera": None,
                "cameras": {},
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("SOARM101_WORKSTATION_CONFIG", str(workstation))

    args = build_parser().parse_args(
        ["read", "--port", "/dev/ttyUSB3", "--robot-id", "explicit", "--json"]
    )
    arm = _arm_from_args(args)

    assert arm.config.port == "/dev/ttyUSB3"
    assert arm.config.robot_id == "explicit"


def test_pose_go_can_request_persistent_torque_on_disconnect() -> None:
    args = build_parser().parse_args(
        [
            "pose",
            "go",
            "agent_start_overhead",
            "--port",
            "/dev/ttyACM9",
            "--robot-id",
            "so101",
            "--yes",
        ]
    )
    arm = _arm_from_args(args, disable_torque_on_disconnect=False)
    assert arm.config.disable_torque_on_disconnect is False


def test_relax_cli_always_requires_enter_confirmation(monkeypatch, capsys) -> None:
    confirmations = 0

    def confirm(*args, **kwargs):
        nonlocal confirmations
        confirmations += 1
        return ""

    monkeypatch.setattr("builtins.input", confirm)
    assert main(["relax", "--simulation"]) == 0
    assert confirmations == 1
    captured = capsys.readouterr()
    assert "Press ENTER to confirm relax" in captured.err
    assert "Follower relaxed." in captured.out


def test_limits_reports_saved_calibration_without_hardware(tmp_path, capsys) -> None:
    from math import degrees

    from soarm101_motion.constants import ALL_MOTORS, JOINT_LIMITS, MOTOR_IDS

    calibration_path = tmp_path / "so101.json"
    calibration_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "source": "test",
                "motors": {
                    name: {
                        "motor_id": MOTOR_IDS[name],
                        "drive_mode": 0,
                        "homing_offset": 0,
                        "range_min": 700,
                        "range_max": 3394,
                    }
                    for name in ALL_MOTORS
                },
            }
        ),
        encoding="utf-8",
    )

    assert (
        main(
            [
                "limits",
                "--robot-id",
                "so101",
                "--calibration",
                str(calibration_path),
                "--json",
            ]
        )
        == 0
    )
    payload = json.loads(capsys.readouterr().out)
    assert payload["calibration_path"] == str(calibration_path)
    assert payload["calibrated_joint_stop_margin_deg"] == pytest.approx(1.0)
    assert payload["calibrated_gripper_stop_margin_deg"] == pytest.approx(1.0)
    assert 0.0 < payload["sleep_gripper"]["normalized"] < 0.1
    assert payload["sleep_gripper"]["raw"] > 700
    assert payload["sleep_gripper"]["calibrated_raw"] == [700, 3394]

    shoulder_calibrated_upper = payload["joints"]["shoulder_pan"]["calibrated_deg"][1]
    shoulder_effective_upper = payload["joints"]["shoulder_pan"]["effective_deg"][1]
    assert shoulder_calibrated_upper > degrees(JOINT_LIMITS["shoulder_pan"][1]) + 1.0
    assert shoulder_effective_upper == pytest.approx(shoulder_calibrated_upper - 1.0)
    assert shoulder_effective_upper > degrees(JOINT_LIMITS["shoulder_pan"][1])

    wrist_calibrated_upper = payload["joints"]["wrist_flex"]["calibrated_deg"][1]
    wrist_effective_upper = payload["joints"]["wrist_flex"]["effective_deg"][1]
    assert wrist_calibrated_upper > degrees(JOINT_LIMITS["wrist_flex"][1]) + 1.0
    assert wrist_effective_upper == pytest.approx(wrist_calibrated_upper - 1.0)
    assert wrist_effective_upper > degrees(JOINT_LIMITS["wrist_flex"][1])

    sleep = payload["sleep_pose_deg"]
    assert sleep["shoulder_pan"] == pytest.approx(0.0)
    assert sleep["shoulder_lift"] == pytest.approx(
        payload["joints"]["shoulder_lift"]["effective_deg"][0]
    )
    assert sleep["elbow_flex"] == pytest.approx(
        payload["joints"]["elbow_flex"]["effective_deg"][1]
    )
    assert sleep["wrist_flex"] == pytest.approx(
        payload["joints"]["wrist_flex"]["effective_deg"][0]
    )
    assert sleep["wrist_roll"] == pytest.approx(0.0)

    assert payload["coarse_cartesian_envelope_mm"]["maximum_tcp_reach"] == pytest.approx(500.0)


def test_sleep_cli_requires_confirmation_and_runs_in_simulation(
    capsys,
    monkeypatch,
) -> None:
    monkeypatch.setattr("builtins.input", lambda *args, **kwargs: "")
    assert main(["sleep", "--simulation"]) == 2
    assert "Refusing to move without --yes" in capsys.readouterr().err

    assert (
        main(
            [
                "sleep",
                "--simulation",
                "--speed-deg-s",
                "8",
                "--acceleration-deg-s2",
                "25",
                "--json",
                "--yes",
            ]
        )
        == 0
    )
    payload = json.loads(capsys.readouterr().out)
    assert payload["accepted"] is True
    assert payload["completed"] is True
    assert payload["final_positions"]["so101_gripper"] == pytest.approx(0.0)
    assert "Press ENTER to relax" in capsys.readouterr().err
