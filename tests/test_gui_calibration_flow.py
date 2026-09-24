"""GUI calibration entry points without touching a serial port."""

from __future__ import annotations

from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

from soarm101_motion.calibration_live import PROVISIONAL_MINIMUM_TRAVEL_TICKS


def test_setup_connects_selected_arm_without_torque(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication

    from soarm101_motion.gui.window import MainWindow

    app = QApplication.instance() or QApplication([])
    window = MainWindow(port="/dev/ttyACM0")
    follower_requests: list[object] = []
    leader_requests: list[object] = []
    window.connect_requested.disconnect(window._worker.connect_robot)
    window.leader_connect_requested.disconnect(window._leader_worker.connect_robot)
    window.connect_requested.connect(follower_requests.append)
    window.leader_connect_requested.connect(leader_requests.append)
    try:
        window.setup_connect_button.click()
        assert len(follower_requests) == 1
        assert follower_requests[0]["port"] == "/dev/ttyACM0"
        assert follower_requests[0]["allow_uncalibrated"] is True
        assert window._follower_setup_session is True

        window._on_connected(True)
        assert not window.enable_button.isEnabled()
        window._on_connected(False)

        window.calibration_target_combo.setCurrentIndex(1)
        window.leader_port_combo.setCurrentText("/dev/ttyACM1")
        window.setup_connect_button.click()
        assert len(leader_requests) == 1
        assert leader_requests[0]["port"] == "/dev/ttyACM1"
        assert leader_requests[0]["allow_uncalibrated"] is True
    finally:
        window.close()
        app.processEvents()


def test_calibration_reports_recording_and_verified_save(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication

    from soarm101_motion.gui.window import MainWindow

    app = QApplication.instance() or QApplication([])
    window = MainWindow(simulation=True)
    dialogs: list[str] = []
    monkeypatch.setattr(
        "soarm101_motion.gui.window.QMessageBox.information",
        lambda _parent, title, _text: dialogs.append(title),
    )
    monkeypatch.setattr(
        "soarm101_motion.gui.window.QTimer.singleShot",
        lambda _delay, _context, callback: callback(),
    )
    progress = {
        name: {
            "travel_ticks": required,
            "required_ticks": required,
            "passed": True,
            "fraction": 1.0,
        }
        for name, required in PROVISIONAL_MINIMUM_TRAVEL_TICKS.items()
    }
    try:
        window._active_calibration_target = "follower"
        progress["shoulder_pan"]["travel_ticks"] = 1500
        progress["shoulder_pan"]["passed"] = False
        window._on_calibration_progress(
            "follower",
            {"phase": "recording", "progress": progress, "duration_s": 30, "remaining_s": 15},
        )
        assert "Shoulder Pan needs 548 more ticks" in window.calibration_status.text()
        assert "15 s remaining" in window.calibration_time_bar.format()

        progress["shoulder_pan"]["travel_ticks"] = 2300
        progress["shoulder_pan"]["passed"] = True
        window._on_calibration_progress(
            "follower",
            {"phase": "recording", "progress": progress, "duration_s": 30, "remaining_s": 5},
        )
        assert "All 6 actuators reached 2/2" in window.calibration_status.text()
        assert "Verifying and saving calibration now" in window.calibration_status.text()
        assert window.calibration_time_bar.format() == "ALL SIX DONE — verifying and saving…"

        window._on_calibration_completed(
            "follower",
            {"path": "/tmp/follower.json", "source": "test", "robot_id": "test"},
        )
        assert window.calibration_time_bar.format() == "CALIBRATION SAVED"
        assert "FOLLOWER CALIBRATION SAVED" in window.calibration_status.text()
        assert dialogs == ["Follower calibration saved"]
    finally:
        window.close()
        app.processEvents()


def test_confirmed_follower_sweep_reaches_worker_signal(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication, QMessageBox

    from soarm101_motion.gui.window import MainWindow

    app = QApplication.instance() or QApplication([])
    window = MainWindow(port="/dev/ttyACM0")
    requests: list[float] = []
    window.calibration_requested.disconnect(window._worker.run_calibration)
    window.calibration_requested.connect(requests.append)
    monkeypatch.setattr(
        "soarm101_motion.gui.window.QMessageBox.question",
        lambda *_args: QMessageBox.StandardButton.Yes,
    )
    try:
        window._connected = True
        window._update_enabled_state()
        assert window.run_calibration_button.isEnabled()
        window.run_calibration_button.click()
        assert requests == [window.calibration_duration.value()]
        assert "Preparing follower motors" in window.calibration_status.text()
        assert "sweep has not started" in window.calibration_time_bar.format()
        assert not window.run_calibration_button.isEnabled()
    finally:
        window.close()
        app.processEvents()


def test_worker_reports_preparation_then_recording() -> None:
    pytest.importorskip("PySide6")
    from soarm101_motion.gui.worker import RobotWorker

    class FakeBackend:
        calibration = SimpleNamespace(source="old-calibration")

        def read_calibration_from_motors(self):
            return self.calibration

        def interactive_calibration(self, *, record_seconds, progress_callback, cancel_event):
            assert record_seconds == 30.0
            assert not cancel_event.is_set()
            progress_callback({})
            return SimpleNamespace(source="fake-sweep")

        def save_calibration(self, _calibration):
            return Path("/tmp/follower.json")

    worker = RobotWorker()
    worker.arm = SimpleNamespace(
        is_connected=True,
        backend=FakeBackend(),
        config=SimpleNamespace(robot_id="so101"),
        get_state=lambda: SimpleNamespace(torque_enabled=False),
        get_joint_limits=lambda: {},
    )
    worker.poll = lambda: None
    phases: list[object] = []
    completed: list[object] = []
    errors: list[str] = []
    worker.calibration_progress.connect(phases.append)
    worker.calibration_completed.connect(completed.append)
    worker.error_message.connect(errors.append)

    worker.run_calibration(30.0)

    assert not errors
    assert [phase["phase"] for phase in phases] == ["preparing", "recording"]
    assert completed[0]["path"] == "/tmp/follower.json"


def test_failed_save_restores_previous_motor_calibration() -> None:
    pytest.importorskip("PySide6")
    from soarm101_motion.gui.worker import RobotWorker

    previous = object()
    replacement = SimpleNamespace(source="new-sweep")

    class FakeBackend:
        calibration = previous

        def read_calibration_from_motors(self):
            return self.calibration

        def interactive_calibration(self, *, record_seconds, progress_callback, cancel_event):
            self.calibration = replacement
            progress_callback({})
            return replacement

        def save_calibration(self, _calibration):
            raise OSError("simulated disk failure")

        def apply_calibration(self, calibration):
            self.calibration = calibration

        def _verify_calibration_matches_motors(self, expected, actual):
            assert actual is expected

    backend = FakeBackend()
    worker = RobotWorker()
    worker.arm = SimpleNamespace(
        is_connected=True,
        backend=backend,
        config=SimpleNamespace(robot_id="so101"),
        get_state=lambda: SimpleNamespace(torque_enabled=False),
    )
    errors: list[str] = []
    worker.error_message.connect(errors.append)
    worker.run_calibration(30.0)

    assert backend.calibration is previous
    assert len(errors) == 1
    assert "previous motor calibration restored" in errors[0]


def test_cancel_button_signals_active_worker_and_reports_restoration(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication
    from soarm101_motion.gui.window import MainWindow

    app = QApplication.instance() or QApplication([])
    window = MainWindow(simulation=True)
    try:
        window._active_calibration_target = "follower"
        window._worker.prepare_calibration()
        window._update_enabled_state()
        assert window.cancel_calibration_button.isEnabled()
        window.cancel_calibration_button.click()
        assert window._worker._calibration_cancel.is_set()
        assert not window.cancel_calibration_button.isEnabled()
        window._on_calibration_cancelled("follower")
        assert window.calibration_time_bar.format() == "CALIBRATION CANCELLED"
        assert "restored" in window.calibration_status.text()
    finally:
        window.close()
        app.processEvents()


def test_calibration_callbacks_run_on_gui_thread(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PySide6")
    from PySide6.QtCore import QThread
    from PySide6.QtWidgets import QApplication
    from soarm101_motion.gui.window import MainWindow

    app = QApplication.instance() or QApplication([])
    window = MainWindow(simulation=True)
    seen: list[QThread] = []
    window._on_calibration_progress = lambda _target, _result: seen.append(QThread.currentThread())
    window._on_calibration_completed = lambda _target, _result: seen.append(QThread.currentThread())
    try:
        def emit_from_worker_thread() -> None:
            window._worker.calibration_progress.emit({"phase": "preparing"})
            window._worker.calibration_completed.emit({})

        thread = threading.Thread(target=emit_from_worker_thread)
        thread.start()
        thread.join()
        app.processEvents()
        assert seen == [app.thread(), app.thread()]
    finally:
        window.close()
        app.processEvents()


def test_all_sweep_gauges_share_visible_row(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication
    from soarm101_motion.constants import ALL_MOTORS
    from soarm101_motion.gui.calibration_progress import CalibrationSweepPanel

    app = QApplication.instance() or QApplication([])
    panel = CalibrationSweepPanel()
    try:
        layout = panel.layout()
        for column, motor in enumerate(ALL_MOTORS):
            assert layout.itemAtPosition(0, column).widget() is panel.gauges[motor]
    finally:
        panel.close()
        app.processEvents()


def test_finished_gauge_pops_once_and_reset_stops_animation(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication
    from soarm101_motion.gui.calibration_progress import SweepGauge

    app = QApplication.instance() or QApplication([])
    gauge = SweepGauge("so101_gripper")
    try:
        gauge.set_progress(
            travel_ticks=1188, required_ticks=900, display_ticks=1129,
            fraction=1.0, passed=False, sweep_number=1,
        )
        assert gauge.pop_progress is None
        gauge.set_progress(
            travel_ticks=1188, required_ticks=900, display_ticks=1129,
            fraction=1.0, passed=True, sweep_number=2, traversals_completed=2,
        )
        assert gauge._pop_timer.isActive()
        gauge._advance_pop()
        progress = gauge.pop_progress
        gauge.set_progress(
            travel_ticks=1188, required_ticks=900, display_ticks=1129,
            fraction=1.0, passed=True, sweep_number=2, traversals_completed=2,
        )
        assert gauge.pop_progress == progress
        gauge.reset_progress()
        assert gauge.pop_progress is None
        assert not gauge._pop_timer.isActive()
    finally:
        gauge.close()
        app.processEvents()


def test_voltage_fault_is_not_reported_as_incomplete_gripper_sweep(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication
    from soarm101_motion.gui.window import MainWindow

    app = QApplication.instance() or QApplication([])
    window = MainWindow(simulation=True)
    monkeypatch.setattr("soarm101_motion.gui.window.QMessageBox.warning", lambda *_: None)
    try:
        window._active_calibration_target = "leader"
        window._on_error(
            "Leader: calibration: read position from so101_gripper: "
            "[ServoStatus] Input voltage error!"
        )
        assert window.calibration_time_bar.format() == "STOPPED — SERVO VOLTAGE FAULT"
        assert "gripper cable" in window.calibration_status.text()
    finally:
        window.close()
        app.processEvents()


def test_switching_to_leader_updates_gripper_gauge_scale(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication
    from soarm101_motion.calibration_live import SWEEP_DISPLAY_TRAVEL_TICKS
    from soarm101_motion.gui.window import MainWindow

    def targets(robot_id: str) -> dict[str, int]:
        values = dict(SWEEP_DISPLAY_TRAVEL_TICKS)
        values["so101_gripper"] = 1129 if robot_id.endswith("-leader") else 1412
        return values

    monkeypatch.setattr("soarm101_motion.gui.window.display_travel_targets", targets)
    app = QApplication.instance() or QApplication([])
    window = MainWindow(simulation=True)
    try:
        gauge = window.calibration_sweep_panel.gauges["so101_gripper"]
        assert gauge.display_ticks == 1412
        window.calibration_target_combo.setCurrentIndex(1)
        assert gauge.display_ticks == 1129
        assert "1129 ticks" in window.calibration_target_note.text()
    finally:
        window.close()
        app.processEvents()
