import pytest

from soarm101_motion.calibration import default_calibration_path
from test_setup_backup import calibration


@pytest.fixture
def window(tmp_path, monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / ".config"))
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication
    from soarm101_motion.gui.window import MainWindow

    app = QApplication.instance() or QApplication([])
    gui = MainWindow(simulation=True)
    yield gui
    gui.close()
    app.processEvents()


def test_calibration_is_on_demand_and_does_not_connect(window):
    requests = []
    window.connect_requested.disconnect(window._worker.connect_robot)
    window.connect_requested.connect(requests.append)
    panel = window.setup_panel
    assert panel.currentIndex() == 0
    panel.open_calibration("leader")
    assert panel.currentIndex() == 1
    assert window.calibration_target_combo.currentData() == "leader"
    assert requests == []
    panel.back()
    assert panel.currentIndex() == 0
    assert not window.tabs.isAncestorOf(window.stop_button)


def test_valid_missing_and_invalid_calibration(window):
    panel = window.setup_panel
    assert not panel.calibration_status("follower")[0]
    path = default_calibration_path("so101")
    calibration().save(path)
    assert panel.calibration_status("follower")[0]
    path.write_text("{broken")
    assert not panel.calibration_status("follower")[0]


def test_saved_calibration_collapses_setup_offer(window):
    calibration().save(default_calibration_path("so101"))
    window.simulation_check.setChecked(False)
    window.setup_panel.refresh()
    label, _, setup = window.setup_panel.cards["follower"]
    assert setup.isHidden()
    assert "Saved calibration found" in label.text()
    assert "Calibration loaded" not in label.text()


def test_offline_model_is_explicit(window):
    window.robot_sidebar.clear_state()
    assert window.robot_sidebar.measurement_label.text() == "Illustration — not live"
    window.robot_sidebar._has_measurement = True
    window.robot_sidebar.clear_state()
    assert "disconnected" in window.robot_sidebar.measurement_label.text()


def test_guidance_can_be_hidden_without_hiding_controls(window):
    window.setup_panel.guided.setChecked(False)
    assert window.walkthrough_label.isHidden()
    assert window.setup_panel.cards["follower"][1].isEnabled()


@pytest.mark.parametrize("dark", [False, True])
def test_theme_text_and_indicator_contrast(window, dark):
    from PySide6.QtGui import QPalette, QColor

    palette = window.palette()
    palette.setColor(QPalette.ColorRole.Window, QColor("#111111" if dark else "#ffffff"))
    window.setPalette(palette)
    window._apply_modern_style()
    palette = window.palette()

    def luminance(color):
        values = [
            v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4
            for v in (color.redF(), color.greenF(), color.blueF())
        ]
        return sum(v * w for v, w in zip(values, (0.2126, 0.7152, 0.0722)))

    def contrast(a, b):
        lo, hi = sorted((luminance(a), luminance(b)))
        return (hi + 0.05) / (lo + 0.05)

    assert (
        contrast(palette.color(QPalette.ColorRole.Text), palette.color(QPalette.ColorRole.Base))
        >= 4.5
    )
    assert (
        contrast(
            palette.color(QPalette.ColorRole.HighlightedText),
            palette.color(QPalette.ColorRole.Highlight),
        )
        >= 4.5
    )
    assert (
        contrast(palette.color(QPalette.ColorRole.Mid), palette.color(QPalette.ColorRole.Base)) >= 3
    )


def test_dark_theme_reaches_child_widgets(window):
    from PySide6.QtGui import QPalette, QColor
    from PySide6.QtWidgets import QApplication

    palette = window.palette()
    palette.setColor(QPalette.ColorRole.Window, QColor("#111111"))
    window.setPalette(palette)
    window._apply_modern_style()
    window.show()
    QApplication.processEvents()
    label = window.setup_panel.cards["follower"][0]
    assert label.palette().color(QPalette.ColorRole.WindowText).lightness() > 200
    assert label.palette().color(QPalette.ColorRole.Base).lightness() < 70


def test_wizard_requires_confirmation_and_skips_existing_calibration(window):
    calibration().save(default_calibration_path("so101"))
    window.simulation_check.setChecked(False)
    window.port_combo.setCurrentText("/dev/test-follower")
    requests = []
    window.connect_requested.disconnect(window._worker.connect_robot)
    window.connect_requested.connect(requests.append)
    panel = window.setup_panel
    panel.setCurrentIndex(4)
    assert panel._guided_step()[2] == "confirm"
    panel._guided_action()
    assert requests == []
    assert panel._guided_step()[2] == "connect"
    panel._guided_action()
    assert len(requests) == 1
    assert requests[0]["allow_uncalibrated"] is False


def test_overview_respects_immediate_connection_gate(window):
    requests = []
    window.connect_requested.disconnect(window._worker.connect_robot)
    window.connect_requested.connect(requests.append)
    button = window.setup_panel.cards["follower"][1]
    # A stale enabled overview button cannot bypass the canonical disabled control.
    window.connect_button.setEnabled(False)
    button.setEnabled(True)
    button.click()
    assert requests == []
