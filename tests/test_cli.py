from soarm101_motion.cli.main import main


def test_info(capsys) -> None:
    assert main(["info"]) == 0
    assert "soarm101-motion-sdk" in capsys.readouterr().out


def test_sim_demo(capsys) -> None:
    assert main(["sim-demo"]) == 0
    assert "Final joints" in capsys.readouterr().out
