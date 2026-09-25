"""Interactive SO-101 kinematic renderer shared by GUI motion workspaces.

The arm centerline and gripper frame come from the SDK's native FK model.  The renderer is
purposefully lightweight: it is a diagnostic/teaching view, not a second physics engine.
"""

from __future__ import annotations

from itertools import pairwise
from math import cos, radians, sin
from typing import Mapping, Sequence

import numpy as np
from PySide6.QtCore import QPointF, QRectF, QSize, Qt
from PySide6.QtGui import (
    QBrush,
    QColor,
    QLinearGradient,
    QMouseEvent,
    QPainter,
    QPalette,
    QPen,
    QWheelEvent,
)
from PySide6.QtWidgets import QWidget

from soarm101_motion.constants import HOME_JOINTS
from soarm101_motion.kinematics import SO101KinematicModel
from soarm101_motion.types import Pose


def _mix(first: QColor, second: QColor, amount: float) -> QColor:
    t = max(0.0, min(1.0, float(amount)))
    return QColor(
        round(first.red() * (1.0 - t) + second.red() * t),
        round(first.green() * (1.0 - t) + second.green() * t),
        round(first.blue() * (1.0 - t) + second.blue() * t),
        round(first.alpha() * (1.0 - t) + second.alpha() * t),
    )


class CartesianArmView(QWidget):
    """Interactive orthographic projection of the SDK kinematic model."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._model = SO101KinematicModel()
        self._joints = dict(HOME_JOINTS)
        self._gripper_position = 1.0
        self._secondary_joints: dict[str, float] | None = None
        self._secondary_gripper = 1.0
        self._secondary_label = "preview"
        self._target_position_m: np.ndarray | None = None
        self._queue_active = False
        self._queue_depth = 0
        self._yaw = 0.0
        self._pitch = 0.0
        self._zoom = 1.0
        self._projection_center = (0.0, 0.0)
        self._projection_scale = 1.0
        self._last_mouse: QPointF | None = None
        self.setMinimumSize(220, 220)
        self.setToolTip(
            "SO-101 kinematic view. Drag to rotate, use the wheel to zoom, "
            "or double-click to return to the side view."
        )

    def sizeHint(self) -> QSize:
        return QSize(360, 340)

    def set_joint_degrees(self, joints_deg: Mapping[str, float]) -> None:
        self._joints = {
            name: radians(float(joints_deg[name]))
            for name in self._model.joint_names
        }
        self.update()

    def set_gripper_position(self, position: float) -> None:
        self._gripper_position = min(1.0, max(0.0, float(position)))
        self.update()

    def set_secondary_joint_degrees(
        self,
        joints_deg: Mapping[str, float],
        *,
        gripper: float = 1.0,
        label: str = "preview",
    ) -> None:
        self._secondary_joints = {
            name: radians(float(joints_deg[name]))
            for name in self._model.joint_names
        }
        self._secondary_gripper = min(1.0, max(0.0, float(gripper)))
        self._secondary_label = str(label)
        self.update()

    def clear_secondary(self) -> None:
        self._secondary_joints = None
        self.update()

    def set_target_xyz_mm(self, xyz_mm: Sequence[float] | None) -> None:
        if xyz_mm is None:
            self._target_position_m = None
        else:
            self._target_position_m = (
                np.asarray(tuple(xyz_mm), dtype=float).reshape(3) / 1000.0
            )
        self.update()

    def set_queue_state(self, *, active: bool, queued: int) -> None:
        self._queue_active = bool(active)
        self._queue_depth = max(0, int(queued))
        self.update()

    def clear_target(self) -> None:
        self.set_target_xyz_mm(None)

    def _gripper_geometry_for(
        self,
        joints: Mapping[str, float],
        gripper_position: float,
    ) -> dict[str, np.ndarray]:
        flange = self._model.forward_matrix(joints, tcp=Pose.identity())
        origin = flange[:3, 3]
        rotation = flange[:3, :3]

        def world(local: Sequence[float]) -> np.ndarray:
            return origin + rotation @ np.asarray(local, dtype=float)

        half_gap = 0.006 + 0.018 * float(gripper_position)
        return {
            "origin": origin.copy(),
            "body_end": world((0.0, 0.0, -0.070)),
            "crossbar_left": world((0.0, -0.028, -0.050)),
            "crossbar_right": world((0.0, 0.028, -0.050)),
            "left_root": world((0.0, -half_gap, -0.052)),
            "left_tip": world((0.0, -half_gap, -0.115)),
            "right_root": world((0.0, half_gap, -0.052)),
            "right_tip": world((0.0, half_gap, -0.115)),
        }

    def gripper_geometry(self) -> dict[str, np.ndarray]:
        """Return primary schematic gripper points in world coordinates."""

        return self._gripper_geometry_for(self._joints, self._gripper_position)

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
            max(1, self.width() - 54) / max(0.05, max_x - min_x),
            max(1, self.height() - 88) / max(0.05, max_z - min_z),
        ) * 0.84

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
        painter.setPen(QPen(color, 1.7))
        painter.drawLine(start, end)
        painter.drawText(end + QPointF(5.0, -4.0), label)

    def _draw_ground_grid(self, painter: QPainter, muted: QColor) -> None:
        painter.setPen(QPen(_mix(muted, QColor(0, 0, 0, 0), 0.60), 0.8))
        extent = 0.18
        step = 0.045
        values = np.arange(-extent, extent + step / 2.0, step)
        for value in values:
            painter.drawLine(
                self._project(np.array([-extent, value, 0.0])),
                self._project(np.array([extent, value, 0.0])),
            )
            painter.drawLine(
                self._project(np.array([value, -extent, 0.0])),
                self._project(np.array([value, extent, 0.0])),
            )

    def _draw_robot(
        self,
        painter: QPainter,
        *,
        joints: Mapping[str, float],
        gripper_position: float,
        link_color: QColor,
        joint_color: QColor,
        ghost: bool = False,
        label: str | None = None,
    ) -> None:
        points = self._model.link_points(joints)
        joint_names = (
            "base",
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
        )
        joint_points = [points[name] for name in joint_names]
        gripper = self._gripper_geometry_for(joints, gripper_position)

        if ghost:
            painter.setPen(
                QPen(
                    link_color,
                    4.0,
                    Qt.PenStyle.DashLine,
                    Qt.PenCapStyle.RoundCap,
                )
            )
            for first, second in pairwise(joint_points):
                painter.drawLine(self._project(first), self._project(second))
        else:
            shadow = QColor(0, 0, 0, 44)
            painter.setPen(
                QPen(shadow, 10.0, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap)
            )
            for first, second in pairwise(joint_points):
                painter.drawLine(
                    self._project(first) + QPointF(1.5, 2.5),
                    self._project(second) + QPointF(1.5, 2.5),
                )
            painter.setPen(
                QPen(
                    link_color,
                    7.0,
                    Qt.PenStyle.SolidLine,
                    Qt.PenCapStyle.RoundCap,
                )
            )
            for first, second in pairwise(joint_points):
                painter.drawLine(self._project(first), self._project(second))

        width = 3.2 if ghost else 5.0
        style = Qt.PenStyle.DashLine if ghost else Qt.PenStyle.SolidLine
        painter.setPen(QPen(link_color, width, style, Qt.PenCapStyle.RoundCap))
        painter.drawLine(
            self._project(gripper["origin"]),
            self._project(gripper["body_end"]),
        )
        painter.drawLine(
            self._project(gripper["crossbar_left"]),
            self._project(gripper["crossbar_right"]),
        )
        finger_width = 2.4 if ghost else 4.0
        painter.setPen(QPen(link_color, finger_width, style, Qt.PenCapStyle.RoundCap))
        painter.drawLine(
            self._project(gripper["left_root"]),
            self._project(gripper["left_tip"]),
        )
        painter.drawLine(
            self._project(gripper["right_root"]),
            self._project(gripper["right_tip"]),
        )

        painter.setBrush(QBrush(joint_color))
        painter.setPen(QPen(link_color, 1.2))
        radius = 3.2 if ghost else 5.0
        for point in joint_points:
            projected = self._project(point)
            painter.drawEllipse(projected, radius, radius)

        if label and ghost:
            tip = self._project(gripper["body_end"])
            painter.setPen(QPen(link_color, 1.0))
            painter.drawText(tip + QPointF(8.0, -8.0), label)

    def paintEvent(self, _event: object) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)

        palette = self.palette()
        base = palette.color(QPalette.ColorRole.Base)
        window = palette.color(QPalette.ColorRole.Window)
        text = palette.color(QPalette.ColorRole.Text)
        muted = palette.color(QPalette.ColorRole.Mid)

        rect = QRectF(self.rect()).adjusted(1.0, 1.0, -1.0, -1.0)
        gradient = QLinearGradient(rect.topLeft(), rect.bottomRight())
        gradient.setColorAt(0.0, _mix(base, window, 0.18))
        gradient.setColorAt(1.0, _mix(base, window, 0.48))
        painter.setPen(QPen(_mix(muted, base, 0.55), 1.0))
        painter.setBrush(QBrush(gradient))
        painter.drawRoundedRect(rect, 16.0, 16.0)

        points = self._model.link_points(self._joints)
        tcp_pose = self._model.forward(self._joints)
        tcp = tcp_pose.position
        gripper = self.gripper_geometry()
        fit_points = [
            *points.values(),
            *gripper.values(),
            np.zeros(3, dtype=float),
            np.array([0.12, 0.0, 0.0]),
            np.array([0.0, 0.12, 0.0]),
            np.array([0.0, 0.0, 0.12]),
        ]

        if self._secondary_joints is not None:
            secondary_points = self._model.link_points(self._secondary_joints)
            secondary_gripper = self._gripper_geometry_for(
                self._secondary_joints,
                self._secondary_gripper,
            )
            fit_points.extend(secondary_points.values())
            fit_points.extend(secondary_gripper.values())
        if self._target_position_m is not None:
            fit_points.append(self._target_position_m)
        self._fit_projection(fit_points)

        self._draw_ground_grid(painter, muted)
        origin = np.zeros(3, dtype=float)
        self._draw_axis(
            painter, origin, np.array([0.10, 0.0, 0.0]), "X", QColor("#ef6a6a")
        )
        self._draw_axis(
            painter, origin, np.array([0.0, 0.10, 0.0]), "Y", QColor("#5dc78b")
        )
        self._draw_axis(
            painter, origin, np.array([0.0, 0.0, 0.10]), "Z", QColor("#69a7ff")
        )

        accent = palette.color(QPalette.ColorRole.Highlight)
        if accent.alpha() == 0:
            accent = QColor("#5d8df7")
        link_color = _mix(text, accent, 0.18)
        joint_color = _mix(base, accent, 0.55)
        ghost_color = QColor(accent)
        ghost_color.setAlpha(118)

        if self._secondary_joints is not None:
            self._draw_robot(
                painter,
                joints=self._secondary_joints,
                gripper_position=self._secondary_gripper,
                link_color=ghost_color,
                joint_color=ghost_color,
                ghost=True,
                label=self._secondary_label,
            )

        self._draw_robot(
            painter,
            joints=self._joints,
            gripper_position=self._gripper_position,
            link_color=link_color,
            joint_color=joint_color,
        )

        tcp_projected = self._project(tcp)
        painter.setBrush(QBrush(QColor("#f2a444")))
        painter.setPen(QPen(QColor("#f7c16b"), 1.2))
        painter.drawEllipse(tcp_projected, 6.0, 6.0)
        painter.setPen(QPen(text, 1.0))
        painter.drawText(tcp_projected + QPointF(9.0, -8.0), "TCP")

        frame_length = 0.035
        frame_colors = (
            QColor("#ef6a6a"),
            QColor("#5dc78b"),
            QColor("#69a7ff"),
        )
        for index, color in enumerate(frame_colors):
            endpoint = tcp + tcp_pose.rotation[:, index] * frame_length
            painter.setPen(QPen(color, 1.5))
            painter.drawLine(self._project(tcp), self._project(endpoint))

        if self._target_position_m is not None:
            target = self._project(self._target_position_m)
            painter.setPen(QPen(QColor("#f2a444"), 2.0))
            size = 8.0
            painter.drawLine(target + QPointF(-size, 0.0), target + QPointF(size, 0.0))
            painter.drawLine(target + QPointF(0.0, -size), target + QPointF(0.0, size))
            painter.drawText(target + QPointF(10.0, -8.0), "target")

        if self._queue_active:
            painter.setPen(QPen(text, 1.0))
            painter.drawText(12, 22, f"{self._queue_depth} Cartesian jogs queued")

        painter.setPen(QPen(_mix(muted, text, 0.20), 1.0))
        painter.drawText(
            12,
            self.height() - 11,
            "Drag rotate  ·  Wheel zoom  ·  Double-click reset",
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
        if self._last_mouse is not None and event.buttons() & Qt.MouseButton.LeftButton:
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
