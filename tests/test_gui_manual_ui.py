from __future__ import annotations

import pytest


@pytest.fixture
def window(monkeypatch, tmp_path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    monkeypatch.setenv("SOARM101_WORKSTATION_CONFIG", str(tmp_path / "workstation.json"))
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication
    import soarm101_motion.workstation as workstation_module
    monkeypatch.setattr(workstation_module, "DEFAULT_CAMERA_CONFIG_PATH", tmp_path / "camera.json")
    from soarm101_motion.gui.window import MainWindow
    app = QApplication.instance() or QApplication([])
    widget = MainWindow(simulation=True)
    yield widget
    widget.close()
    app.processEvents()


def test_live_manual_is_opt_in_and_rate_presets_keep_numeric_customization(window) -> None:
    assert not window.live_manual_check.isChecked()
    assert not window._live_manual_active
    window.joint_speed_preset.setCurrentIndex(window.joint_speed_preset.findData("medium"))
    assert window.joint_speed.value() == pytest.approx(50.0)
    assert window.joint_acceleration.value() == pytest.approx(500.0)
    assert window.joint_speed_preset.currentData() == "medium"
    window.joint_speed.setValue(42.0)
    assert window.joint_speed_preset.currentData() == "custom"


def test_live_manual_locks_rates_and_edit_mode_and_ignores_late_ready(window) -> None:
    window._live_manual_active = True
    window.live_manual_check.setChecked(True)
    window._update_enabled_state()
    assert not window.joint_speed.isEnabled()
    assert not window.joint_acceleration.isEnabled()
    assert not window.joint_speed_preset.isEnabled()
    window.edit_joint_targets_check.setChecked(True)
    window.edit_joint_targets_check.setChecked(False)
    assert window.edit_joint_targets_check.isChecked()
    before = {name: spin.value() for name, spin in window.joint_spins.items()}
    window._live_manual_active = False
    window._on_live_manual_ready({name: 73.0 for name in before})
    assert {name: spin.value() for name, spin in window.joint_spins.items()} == before


def test_live_manual_end_loads_fresh_measured_pose_instead_of_old_idle_target(window) -> None:
    window._live_manual_active = True
    window.live_manual_check.setChecked(True)
    window._live_manual_idle_targets = {name: 0.0 for name in window.joint_spins}
    measured = {name: 3.0 for name in window.joint_spins}
    measured["wrist_roll"] = 12.0
    window._on_live_manual_ended({"joints_deg": measured, "moved": True})
    assert not window._live_manual_active
    assert not window.live_manual_check.isChecked()
    assert window.joint_spins["wrist_roll"].value() == pytest.approx(12.0)


def test_live_manual_ready_seeds_gripper_without_publishing_startup_feedback(window) -> None:
    window._live_manual_active = True
    sent = []
    window._worker.publish_live_manual_target = sent.append
    joints = {name: 4.0 for name in window.joint_spins}
    window._on_live_manual_ready({"joints_deg": joints, "gripper": 0.37})
    assert window.gripper_spin.value() == pytest.approx(0.37)
    assert window.gripper_slider.value() == 370
    assert sent == []


def test_live_manual_gripper_edit_publishes_only_gripper_target(window) -> None:
    window._live_manual_active = True
    window._live_manual_ready_state = True
    sent = []
    window._worker.publish_live_manual_target = sent.append
    window.gripper_spin.setValue(0.62)
    assert sent[-1]["gripper"] == pytest.approx(0.62)
    assert not any(name in sent[-1] for name in window.joint_spins)


def test_ready_and_single_joint_edit_preserve_exact_worker_targets(window) -> None:
    window._live_manual_active = True
    window._live_manual_session_id = 7
    sent = []
    window._worker.publish_live_manual_target = sent.append
    joints = {name: 12.34567 for name in window.joint_spins}
    window._on_live_manual_ready({"session_id": 7, "joints_deg": joints, "gripper": 0.12345})
    assert sent == []
    window.joint_spins["wrist_flex"].setValue(14.0)
    assert sent[-1] == {"session_id": 7.0, "wrist_flex": 14.0}
    assert window.joint_spins["shoulder_pan"].value() == pytest.approx(12.3)


def test_stale_ready_and_ended_do_not_change_reenabled_session(window) -> None:
    window._live_manual_active = True
    window._live_manual_session_id = 22
    window._live_manual_ready_state = False
    window._on_live_manual_ready({"session_id": 21, "joints_deg": {name: 75 for name in window.joint_spins}})
    assert not window._live_manual_ready_state
    window._on_live_manual_ended({"session_id": 21, "joints_deg": {name: 0 for name in window.joint_spins}})
    assert window._live_manual_active
    assert window.live_manual_check.isChecked() is False


def test_live_manual_preserves_unsubmitted_edit_only_if_no_motion_occurred(window) -> None:
    window._live_manual_active = True
    window.live_manual_check.setChecked(True)
    window._live_manual_idle_targets = {name: 0.0 for name in window.joint_spins}
    measured = {name: 2.0 for name in window.joint_spins}
    window._on_live_manual_ended({"joints_deg": measured, "moved": False})
    assert all(spin.value() == pytest.approx(0.0) for spin in window.joint_spins.values())
