import json

from soarm101_motion.cli.main import build_parser, main
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
