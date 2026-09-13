from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def test_effort_characterization_help_runs_without_hardware() -> None:
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [sys.executable, "examples/effort_safety_characterization.py", "--help"],
        cwd=root,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "--motor-current-trip" in result.stdout
    assert "--feedback-interval" in result.stdout
