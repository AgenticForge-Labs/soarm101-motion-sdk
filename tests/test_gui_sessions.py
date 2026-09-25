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


def test_manual_workspace_keeps_gripper_visible_across_arm_modes(window):
    labels = [
        window.manual_mode_tabs.tabText(i)
        for i in range(window.manual_mode_tabs.count())
    ]
    assert labels == ["Joint / angular", "Cartesian"]
    assert window.manual_page.isAncestorOf(window.gripper_slider)
    assert not window.manual_mode_tabs.isAncestorOf(window.gripper_slider)
    assert len(window._coordination_park_buttons) == 2
    assert len(window._coordination_relink_buttons) == 2


def test_gripper_speed_preset_is_shared_by_manual_teleop_edit_and_run(window):
    fast_index = window.manual_gripper_speed_combo.findData(5.0)
    assert fast_index >= 0
    window.manual_gripper_speed_combo.setCurrentIndex(fast_index)
    assert window._gripper_speed_multiplier == 5.0
    for combo in (
        window.manual_gripper_speed_combo,
        window.teleop_gripper_speed_combo,
        window.trajectory_gripper_speed_combo,
        window.run_gripper_speed_combo,
    ):
        assert float(combo.currentData()) == 5.0


def test_manual_kinematic_view_draws_all_links_and_gripper_in_frame(window, capfd):
    from math import radians

    window.resize(1440, 900)
    window.show()
    window.tabs.setCurrentWidget(window.manual_page)
    view = window.cartesian_view
    view.set_joint_degrees({
        "shoulder_pan": -8.2,
        "shoulder_lift": 4.3,
        "elbow_flex": -1.4,
        "wrist_flex": -81.4,
        "wrist_roll": -79.1,
    })
    view.grab()

    joints = {name: radians(value) for name, value in {
        "shoulder_pan": -8.2,
        "shoulder_lift": 4.3,
        "elbow_flex": -1.4,
        "wrist_flex": -81.4,
        "wrist_roll": -79.1,
    }.items()}
    for point in view._model.link_points(joints).values():
        projected = view._project(point)
        assert 0 < projected.x() < view.width()
        assert 0 < projected.y() < view.height()
    for point in view.gripper_geometry().values():
        projected = view._project(point)
        assert 0 < projected.x() < view.width()
        assert 0 < projected.y() < view.height()
    assert "Error calling Python override" not in capfd.readouterr().err


def test_kinematic_view_gripper_uses_wrist_frame_and_changes_aperture(window):
    import numpy as np
    from soarm101_motion.types import Pose

    view = window.cartesian_view
    view.set_joint_degrees({name: 0.0 for name in view._model.joint_names})

    view.set_gripper_position(0.0)
    closed = view.gripper_geometry()
    view.set_gripper_position(1.0)
    opened = view.gripper_geometry()

    expected_origin = view._model.forward_matrix(
        {name: 0.0 for name in view._model.joint_names},
        tcp=Pose.identity(),
    )[:3, 3]
    assert closed["origin"] == pytest.approx(expected_origin)
    closed_gap = np.linalg.norm(closed["left_tip"] - closed["right_tip"])
    open_gap = np.linalg.norm(opened["left_tip"] - opened["right_tip"])
    assert open_gap > closed_gap


def test_leader_park_release_and_sync_capture_are_explicit(window):
    park_requests = []
    release_requests = []
    sync_captures = []
    window.leader_park_requested.disconnect(window._leader_worker.enable)
    window.leader_release_requested.disconnect(window._leader_worker.relax)
    window.capture_follower_pose_requested.disconnect(window._worker.capture_measured_pose)
    window.leader_park_requested.connect(lambda: park_requests.append(True))
    window.leader_release_requested.connect(lambda: release_requests.append(True))
    window.capture_follower_pose_requested.connect(sync_captures.append)

    window._leader_connected = True
    window._leader_torque_enabled = False
    window._toggle_leader_park()
    assert park_requests == [True]

    window._leader_torque_enabled = True
    window._toggle_leader_park()
    assert release_requests == [True]

    window._sync_capture_pending = None
    window._move_leader_to_follower()
    assert sync_captures == [
        {"kind": "sync", "source": "follower", "destination": "leader"}
    ]


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
