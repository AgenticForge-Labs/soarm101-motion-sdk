"""Interactive kinematic view for Cartesian control.

This intentionally uses the SDK's native FK model rather than a second renderer-specific
robot model. It gives the GUI a lightweight 3D view with no dependency beyond PySide6
and NumPy, while keeping the displayed joint/TCP geometry aligned with planning.
"""

from __future__ import annotations

from itertools import pairwise
from math import cos, radians, sin
from typing import Mapping, Sequence

import numpy as np
from PySide6.QtCore import QPointF, QSize, Qt
from PySide6.QtGui import QColor, QMouseEvent, QPainter, QPalette, QPen, QWheelEvent
from PySide6.QtWidgets import QWidget

from soarm101_motion.constants import HOME_JOINTS
from soarm101_motion.kinematics import SO101KinematicModel


class CartesianArmView(QWidget):
    """Small interactive 3D projection of the SDK kinematic model."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._model = SO101KinematicModel()
        self._joints = dict(HOME_JOINTS)
        self._target_position_m: np.ndarray | None = None
        self._queue_active = False
        self._queue_depth = 0
        # Start orthographic in the arm's X/Z plane so the main link spans are
        # comparable to a physical side view without perspective shortening.
        self._yaw = 0.0
        self._pitch = 0.0
        self._zoom = 1.0
        self._projection_center = (0.0, 0.0)
        self._projection_scale = 1.0
        self._last_mouse: QPointF | None = None
        self.setMinimumSize(300, 300)
        self.setToolTip(
            "Joint-center diagram from the SO-101 kinematic model. Drag to rotate, "
            "use the wheel to zoom, or double-click for the side view."
        )

    def sizeHint(self) -> QSize:
        return QSize(380, 380)

    def set_joint_degrees(self, joints_deg: Mapping[str, float]) -> None:
        self._joints = {
            name: radians(float(joints_deg[name]))
            for name in self._model.joint_names
        }
        self.update()

    def set_target_xyz_mm(self, xyz_mm: Sequence[float] | None) -> None:
        if xyz_mm is None:
            self._target_position_m = None
        else:
            self._target_position_m = np.asarray(tuple(xyz_mm), dtype=float).reshape(3) / 1000.0
        self.update()

    def set_queue_state(self, *, active: bool, queued: int) -> None:
        self._queue_active = bool(active)
        self._queue_depth = max(0, int(queued))
        self.update()

    def clear_target(self) -> None:
        self.set_target_xyz_mm(None)

    def _view_coordinates(self, point: np.ndarray) -> tuple[float, float, float]:
        x, y, z = (float(value) for value in point)

        cy = cos(self._yaw)
        sy = sin(self._yaw)
        x1 = cy * x - sy * y
        y1 = sy * x + cy * y
        z1 = z

        cp = cos(self._pitch)
        sp = sin(self._pitch)
        y2 = cp * y1 - sp * z1
        z2 = sp * y1 + cp * z1
        return x1, y2, z2

    def _project(self, point: np.ndarray) -> QPointF:
        x, _depth, z = self._view_coordinates(point)
        center_x, center_z = self._projection_center
        scale = self._projection_scale * self._zoom
        return QPointF(
            self.width() * 0.50 + (x - center_x) * scale,
            self.height() * 0.50 - (z - center_z) * scale,
        )

    def _fit_projection(self, points: Sequence[np.ndarray]) -> None:
        projected = [self._view_coordinates(point) for point in points]
        min_x = min(point[0] for point in projected)
        max_x = max(point[0] for point in projected)
        min_z = min(point[2] for point in projected)
        max_z = max(point[2] for point in projected)
        self._projection_center = ((min_x + max_x) / 2.0, (min_z + max_z) / 2.0)
        self._projection_scale = min(
            max(1, self.width() - 48) / max(0.05, max_x - min_x),
            max(1, self.height() - 96) / max(0.05, max_z - min_z),
        ) * 0.85

    def _draw_axis(
        self,
        painter: QPainter,
        origin: np.ndarray,
        direction: np.ndarray,
        label: str,
        color: QColor,
    ) -> None:
        start = self._project(origin)
        end = self._project(origin + direction)
        painter.setPen(QPen(color, 2.2))
        painter.drawLine(start, end)
        painter.drawText(end + QPointF(5.0, -4.0), label)

    def paintEvent(self, _event: object) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        painter.fillRect(
            self.rect(),
            self.palette().brush(QPalette.ColorRole.Base),
        )

        text_color = self.palette().color(QPalette.ColorRole.Text)
        muted = self.palette().color(QPalette.ColorRole.Mid)

        points = self._model.link_points(self._joints)
        ordered_names = (
            "base",
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "tcp",
        )
        ordered = [points[name] for name in ordered_names]
        tcp_pose = self._model.forward(self._joints)
        tcp = tcp_pose.position
        frame_length = 0.035
        fit_points = [*ordered, np.zeros(3, dtype=float)]
        fit_points.extend(np.eye(3) * 0.10)
        fit_points.extend(tcp + tcp_pose.rotation[:, index] * frame_length for index in range(3))
        if self._target_position_m is not None:
            fit_points.append(self._target_position_m)
        self._fit_projection(fit_points)

        origin = np.zeros(3, dtype=float)
        self._draw_axis(painter, origin, np.array([0.10, 0.0, 0.0]), "X", QColor("#d95c5c"))
        self._draw_axis(painter, origin, np.array([0.0, 0.10, 0.0]), "Y", QColor("#55a868"))
        self._draw_axis(painter, origin, np.array([0.0, 0.0, 0.10]), "Z", QColor("#4c78a8"))

        painter.setPen(QPen(text_color, 6.0, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        for first, second in pairwise(ordered):
            painter.drawLine(self._project(first), self._project(second))

        painter.setPen(QPen(muted, 1.5))
        painter.setBrush(text_color)
        for name, point in zip(ordered_names, ordered, strict=True):
            projected = self._project(point)
            radius = 4.5 if name != "tcp" else 6.0
            painter.drawEllipse(projected, radius, radius)

        frame_colors = (QColor("#d95c5c"), QColor("#55a868"), QColor("#4c78a8"))
        for index, color in enumerate(frame_colors):
            endpoint = tcp + tcp_pose.rotation[:, index] * frame_length
            painter.setPen(QPen(color, 1.6))
            painter.drawLine(self._project(tcp), self._project(endpoint))

        if self._target_position_m is not None:
            target = self._project(self._target_position_m)
            painter.setPen(QPen(QColor("#d08b28"), 2.2))
            size = 8.0
            painter.drawLine(target + QPointF(-size, 0.0), target + QPointF(size, 0.0))
            painter.drawLine(target + QPointF(0.0, -size), target + QPointF(0.0, size))
            painter.drawText(target + QPointF(10.0, -8.0), "target")

        painter.setPen(text_color)
        queue_text = (
            f"Cartesian queue: {self._queue_depth} waiting"
            if self._queue_active
            else "Cartesian queue: idle"
        )
        painter.drawText(10, 20, queue_text)
        painter.setPen(muted)
        painter.drawText(
            10, self.height() - 10,
            "Drag: rotate · Wheel: zoom · Double-click: side view · world/base XYZ",
        )
        painter.end()

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self._yaw = 0.0
            self._pitch = 0.0
            self._zoom = 1.0
            self.update()
            event.accept()
            return
        super().mouseDoubleClickEvent(event)

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self._last_mouse = event.position()
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if (
            self._last_mouse is not None
            and event.buttons() & Qt.MouseButton.LeftButton
        ):
            delta = event.position() - self._last_mouse
            self._last_mouse = event.position()
            self._yaw += float(delta.x()) * 0.010
            self._pitch = max(
                radians(-80.0),
                min(radians(80.0), self._pitch + float(delta.y()) * 0.008),
            )
            self.update()
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self._last_mouse = None
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event: QWheelEvent) -> None:
        factor = 1.12 if event.angleDelta().y() > 0 else 1.0 / 1.12
        self._zoom = max(0.55, min(2.5, self._zoom * factor))
        self.update()
        event.accept()
