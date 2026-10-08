"""Keep tests from writing calibration snapshots into a developer's config dir."""

from __future__ import annotations

import pytest


@pytest.fixture(autouse=True)
def isolate_calibration_history(monkeypatch: pytest.MonkeyPatch, tmp_path) -> None:
    from soarm101_motion import calibration

    monkeypatch.setattr(
        calibration,
        "default_calibration_history_dir",
        lambda robot_id: tmp_path / "calibration-history" / str(robot_id),
    )
