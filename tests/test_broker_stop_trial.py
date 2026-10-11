"""Static operator-gate checks for the physically attended STOP trial.

The actual trial is NEVER invoked with a terminal or physical port in CI.
"""
from __future__ import annotations

import importlib.util
import io
from pathlib import Path

import pytest


def _load_trial(monkeypatch):
    examples = Path(__file__).resolve().parents[1] / "examples"
    monkeypatch.syspath_prepend(str(examples))
    script = examples / "persistent_broker_stop_trial.py"
    spec = importlib.util.spec_from_file_location("test_stop_physical_trial", script)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_stop_trial_never_supports_unattended_yes_and_requires_tty(
    tmp_path, monkeypatch,
) -> None:
    trial = _load_trial(monkeypatch)
    args = ["--port", "/dev/ttyACM1", "--output", str(tmp_path / "trace.jsonl")]
    with pytest.raises(SystemExit):
        trial.build_parser().parse_args([*args, "--yes"])
    monkeypatch.setattr(trial.sys, "stdin", io.StringIO(""))
    with pytest.raises(PermissionError, match="interactive operator terminal"):
        trial.main(args)
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize("argument", [
    ("--speed-deg-s", "nan"),
    ("--acceleration-deg-s2", "-1"),
    ("--command-frequency-hz", "inf"),
])
def test_stop_trial_rejects_nonfinite_or_nonpositive_rates_without_hardware(
    tmp_path, monkeypatch, argument,
) -> None:
    trial = _load_trial(monkeypatch)
    monkeypatch.setattr(trial.sys, "stdin", type("TTY", (), {"isatty": lambda self: True})())
    args = trial.build_parser().parse_args([
        "--port", "/dev/ttyACM1",
        "--output", str(tmp_path / "trace.jsonl"), *argument,
    ])
    with pytest.raises(ValueError, match="finite and positive"):
        trial._require_operator(args)
    assert not list(tmp_path.iterdir())


def test_stop_trial_refuses_non_dev_port_before_creating_output(
    tmp_path, monkeypatch,
) -> None:
    trial = _load_trial(monkeypatch)
    monkeypatch.setattr(trial.sys, "stdin", type("TTY", (), {"isatty": lambda self: True})())
    with pytest.raises(ValueError, match="Linux /dev/ tty"):
        trial.main([
            "--port", "FAKE", "--output", str(tmp_path / "trace.jsonl"),
        ])
    assert not list(tmp_path.iterdir())
