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


def test_offline_illustration_uses_neutral_gripper_opening(view):
    assert view._gripper_position == pytest.approx(0.45)
    assert 0.0 < view._gripper_position < 1.0


def test_presentation_body_width_is_restrained_and_bounded():
    pytest.importorskip("PySide6")
    from soarm101_motion.gui.cartesian_view import _presentation_body_width

    assert _presentation_body_width(0.035, 500.0, ghost=False) == pytest.approx(10.85)
    assert _presentation_body_width(0.035, 500.0, ghost=True) == pytest.approx(8.05)
    assert _presentation_body_width(0.090, 1000.0, ghost=False) == pytest.approx(28.0)
    assert _presentation_body_width(0.001, 500.0, ghost=False) == pytest.approx(4.0)


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


def test_standard_presets_keep_zoom_scale_and_world_orbit_center(view):
    view._zoom = 1.6
    view.grab()
    scale = view._projection_scale * view._zoom
    center = view._orbit_center_world.copy()
    for preset in ("Front", "Top", "Isometric", "Side"):
        view.set_standard_view(preset)
        view.grab()
        assert view._projection_scale * view._zoom == pytest.approx(scale)
        assert view._project(center).x() == pytest.approx(view.width() * 0.5)
        assert view._project(center).y() == pytest.approx(view.height() * 0.5 + 10.0)
    view.fit_button.click()
    view.grab()
    assert view._zoom == pytest.approx(1.0)


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


@pytest.mark.parametrize('yaw,pitch', [(0., 0.), (0.7, 0.4), (-1.2, 1.57)])
def test_mesh_projection_matches_joint_and_tcp_markers(view, yaw, pitch):
    view._yaw, view._pitch = yaw, pitch
    vertices = np.array([[[0., 0., 0.], [0.2, -0.1, 0.3], [-0.1, 0.2, 0.05]]])
    xs, ys = view._project_vertices(vertices)
    for index, vertex in enumerate(vertices[0]):
        point = view._project(vertex)
        assert xs[0, index] == pytest.approx(point.x())
        assert ys[0, index] == pytest.approx(point.y())


def test_overlapping_mesh_triangles_remain_opaque(view, monkeypatch):
    from PySide6.QtCore import Qt
    from PySide6.QtGui import QColor, QImage, QPainter
    import soarm101_motion.gui.cartesian_view as module

    tri = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.0, 0.0, 0.1]])
    monkeypatch.setattr(module, 'world_triangles', lambda *a, **kw: [
        (np.array([tri, tri[::-1]]), '3d_printed')
    ])
    view._yaw = view._pitch = 0.0
    view._projection_center = (0.0, 0.0)
    view._projection_scale = 1000.0
    view._zoom = 1.0
    view._mesh_image_key = None
    image = QImage(view.size(), QImage.Format.Format_ARGB32_Premultiplied)
    image.fill(Qt.GlobalColor.transparent)
    painter = QPainter(image)
    view._draw_meshes(painter, view._joints, QColor('gray'), view._gripper_position)
    painter.end()
    point = view._project(np.array([0.025, 0.0, 0.025])).toPoint()
    assert image.pixelColor(point).alpha() > 0


def test_identical_feedback_does_not_restart_detail_timer(view):
    from math import degrees

    values = {name: degrees(value) for name, value in view._joints.items()}
    view._restore_full_detail()
    view.set_joint_degrees(values)
    view.set_gripper_position(view._gripper_position)
    assert not view._interactive_detail
    assert not view._detail_timer.isActive()
    values['shoulder_pan'] += 1.0
    view.set_joint_degrees(values)
    assert view._interactive_detail
    assert view._detail_timer.isActive()
