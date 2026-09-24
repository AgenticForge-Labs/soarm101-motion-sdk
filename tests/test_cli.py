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
