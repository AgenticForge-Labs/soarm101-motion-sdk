"""Session layout and discovery must never implicitly connect or enable an arm."""
import pytest


@pytest.fixture
def window(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication
    from soarm101_motion.gui.window import MainWindow
    app = QApplication.instance() or QApplication([])
    gui = MainWindow(simulation=True)
    yield gui
    gui.close()
    app.processEvents()


def test_roles_are_in_setup_tab(window):
    assert window.calibration_page.isAncestorOf(window.port_combo)
    assert window.calibration_page.isAncestorOf(window.leader_port_combo)
    assert not window.tabs.isAncestorOf(window.stop_button)
    labels = [window.tabs.tabText(i) for i in range(window.tabs.count())]
    assert labels == ["Setup", "Manual", "Teleoperation", "Record / Teach", "Edit recordings", "Run", "Log"]
    assert window.teleop_page.isAncestorOf(window.teleop_button)
    assert window.record_page.isAncestorOf(window.save_point_button)
    assert window.record_page.isAncestorOf(window.record_button)


def test_discovery_selects_roles_but_never_connects(window):
    requests = []
    window.connect_requested.disconnect(window._worker.connect_robot)
    window.leader_connect_requested.disconnect(window._leader_worker.connect_robot)
    window.connect_requested.connect(requests.append)
    window.leader_connect_requested.connect(requests.append)
    window.enable_requested.connect(lambda: requests.append("enable"))
    window._on_arm_discovery_completed([
        dict(port="/dev/follower", role="follower", voltage_v=12.0, status="ok", motor_count=6),
        dict(port="/dev/leader", role="leader", voltage_v=5.0, status="ok", motor_count=6),
    ])
    assert window.port_combo.currentText() == "/dev/follower"
    assert window.leader_port_combo.currentText() == "/dev/leader"
    assert requests == []
    window._on_arm_discovery_completed([
        dict(port="/dev/follower", role="follower", voltage_v=12.0, status="ok", motor_count=6),
    ])
    assert window.leader_port_combo.currentText() == ""


def test_same_device_cannot_connect_as_both_roles(window):
    window.simulation_check.setChecked(False)
    window.leader_simulation_check.setChecked(False)
    window.port_combo.setCurrentText("/dev/same")
    window.leader_port_combo.setCurrentText("/dev/same")
    requests = []
    window.connect_requested.disconnect(window._worker.connect_robot)
    window.leader_connect_requested.disconnect(window._leader_worker.connect_robot)
    window.connect_requested.connect(requests.append)
    window.leader_connect_requested.connect(requests.append)
    window._toggle_connection()
    window._toggle_leader_connection()
    assert requests == []
    assert "Choose different devices" in window.log.toPlainText()


def test_error_marks_log_tab_and_teleop_status(window):
    window.show()
    window._on_error("live teleoperation: streamed joint acceleration exceeded limit")
    assert window.tabs.tabText(window.tabs.indexOf(window.log_page)) == "Log •"
    assert window.alert_label.isVisible()
    assert "acceleration exceeded" in window.teleop_status.text()
