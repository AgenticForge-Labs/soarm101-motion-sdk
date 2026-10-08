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


@pytest.fixture(scope="session", autouse=True)
def retain_qt_application_for_test_session():
    """Keep QApplication alive until all GUI widgets and QThreads are finished.

    Several GUI tests create a QApplication as a function-local temporary.
    Losing Python's last strong reference can destroy the Qt application while
    other Shiboken-owned QObject wrappers survive, resulting in a C++ abort
    *after* pytest reports that the suite passed. The session fixture avoids
    constructing and destroying QApplication repeatedly. The core package
    remains testable without the optional GUI dependency.
    """
    import importlib.util
    import os

    if importlib.util.find_spec("PySide6") is None:
        yield
        return

    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication

    app = QApplication.instance() or QApplication([])
    yield app
    # Deliberately retain 'app' until pytest has finished tearing down tests.
    app.processEvents()
