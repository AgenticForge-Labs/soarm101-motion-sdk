"""Stable display scale and session TCP must survive model preview updates."""

import numpy as np
import pytest


@pytest.fixture
def view(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication
    from soarm101_motion.gui.cartesian_view import CartesianArmView

    app = QApplication.instance() or QApplication([])
    widget = CartesianArmView()
    widget.resize(380, 340)
    widget.show()
    app.processEvents()
    yield widget
    widget.close()


def test_pose_preview_and_target_updates_keep_fixed_projection(view):
    view.grab()
    scale, center = view._projection_scale, view._projection_center
    view.set_joint_degrees(dict(zip(view._model.joint_names, (30, -45, 50, 40, 90))))
    view.set_gripper_position(0.0)
    view.set_target_xyz_mm((800, 300, 100))
    view.set_secondary_joint_degrees({name: 10 for name in view._model.joint_names})
    view.grab()
    assert view._projection_scale == scale
    assert view._projection_center == center
    view.fit_button.click()
    view.grab()
    assert view._projection_scale < scale
    view.auto_fit.setChecked(True)
    view.clear_target()
    view.grab()
    assert view._projection_scale > scale / 2


@pytest.mark.parametrize("preset,depth_axis", [("Side", 1), ("Front", 0), ("Top", 2)])
def test_standard_views_hide_only_the_expected_depth_axis(view, preset, depth_axis):
    view.view_combo.setCurrentText(preset)
    view.grab()
    origin = view._project(np.zeros(3))
    for axis in range(3):
        delta = view._project(np.eye(3)[axis] * 0.05) - origin
        length = np.hypot(delta.x(), delta.y())
        assert length == pytest.approx(
            0 if axis == depth_axis else 0.05 * view._projection_scale, abs=1e-8
        )


def test_sidebar_uses_active_session_tcp(view):
    from soarm101_motion.gui.arm_status import RobotStatusPanel
    from soarm101_motion.types import Pose

    panel = RobotStatusPanel()
    tcp = Pose.from_xyz_rpy(0.03, 0.01, -0.15, 0, 0, 0)
    panel.update_state({
        "joints_deg": {name: 0.0 for name in view._model.joint_names},
        "tcp_xyz_rpy": tcp.xyz_rpy(),
        "gripper": 0.5, "pose_mm_deg": (0, 0, 0, 0, 0, 0), "connected": True,
    })
    assert panel.view._model.tcp.as_matrix() == pytest.approx(tcp.as_matrix())
    panel.close()


def test_worker_pose_and_drawing_share_one_joint_snapshot(view, monkeypatch):
    from math import degrees
    from soarm101_motion import SOARM101
    from soarm101_motion.gui.worker import RobotWorker

    with SOARM101.simulated() as arm:
        worker = RobotWorker()
        worker.arm = arm
        worker._simulation = True
        payloads = []
        worker.state_changed.connect(payloads.append)
        errors = []
        worker.error_message.connect(errors.append)
        snapshot = arm.get_joint_positions().positions

        def unexpected_second_read(*args, **kwargs):
            raise AssertionError("GUI FK must not reread moving joints")

        monkeypatch.setattr(arm, "get_position", unexpected_second_read)
        worker.poll()
        assert not errors
        assert len(payloads) == 1
        payload = payloads[0]
        expected = arm.model.forward(snapshot, tcp=arm.active_tcp).xyz_rpy()
        assert payload["joints_deg"] == pytest.approx(
            {name: degrees(value) for name, value in snapshot.items()}
        )
        assert payload["pose_mm_deg"] == pytest.approx(
            tuple(value * 1000 for value in expected[:3])
            + tuple(degrees(value) for value in expected[3:])
        )
        assert payload["tcp_xyz_rpy"] == pytest.approx(arm.active_tcp.xyz_rpy())
        worker.arm = None
