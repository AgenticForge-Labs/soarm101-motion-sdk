"""Interactive SO-101 kinematic renderer shared by GUI motion workspaces.

FK and joint markers come from the SDK's native kinematic model. The visible arm bodies use
presentation-only centerlines derived from the packaged URDF visual primitives; coarse safety
centerlines remain separate. The renderer is a diagnostic/teaching view, not a physics engine
or certified collision model.
"""

from __future__ import annotations

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
from PySide6.QtWidgets import QCheckBox, QComboBox, QHBoxLayout, QPushButton, QVBoxLayout, QWidget

from soarm101_motion.constants import HOME_JOINTS
from soarm101_motion.kinematics import SO101KinematicModel
from soarm101_motion.types import Pose


_OFFLINE_GRIPPER_POSITION = 0.45


def _presentation_body_width(thickness_m: float, scale: float, *, ghost: bool) -> float:
    """Map nominal URDF thickness to a restrained 2D schematic stroke.

    The packaged URDF omits servo housings and printed brackets, so raw primitive
    thickness is not a faithful proxy for all visible robot bulk. Keep the visual
    bodies readable without turning missing hardware into oversized round blobs.
    """

    floor = 3.0 if ghost else 4.0
    ceiling = 18.0 if ghost else 28.0
    factor = 0.46 if ghost else 0.62
    return max(floor, min(ceiling, float(thickness_m) * float(scale) * factor))


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
        self._gripper_position = _OFFLINE_GRIPPER_POSITION
        self._secondary_joints: dict[str, float] | None = None
        self._secondary_gripper = _OFFLINE_GRIPPER_POSITION
        self._secondary_label = "preview"
        self._target_position_m: np.ndarray | None = None
        self._queue_active = False
        self._queue_depth = 0
        self._yaw = 0.0
        self._pitch = 0.0
        self._zoom = 1.0
        self._projection_center = (0.0, 0.0)
        self._projection_scale = 1.0
        self._projection_ready = False
        self._projection_size = (0, 0)
        self._last_mouse: QPointF | None = None
        self.setMinimumSize(220, 220)
        self.setToolTip(
            "Nominal SO-101 URDF-visual and jaw schematic in model/base coordinates. "
            "The grid is model Z=0, not a measured table. Drag to rotate, wheel to zoom."
        )
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        toolbar = QHBoxLayout()
        toolbar.setSpacing(5)
        self.view_combo = QComboBox()
        self.view_combo.addItems(["Side", "Front", "Top", "Isometric", "Free"])
        self.view_combo.setToolTip("Standard orthographic model views")
        self.view_combo.currentTextChanged.connect(self.set_standard_view)
        toolbar.addWidget(self.view_combo)
        self.fit_button = QPushButton("Fit")
        self.fit_button.setToolTip("Fit the current arm, preview, and target once")
        self.fit_button.clicked.connect(self.fit_view)
        toolbar.addWidget(self.fit_button)
        self.auto_fit = QCheckBox("Auto fit")
        self.auto_fit.setToolTip("Continuously rescale; leave off to compare movement at a fixed scale")
        self.auto_fit.toggled.connect(lambda _checked: self.update())
        toolbar.addWidget(self.auto_fit)
        toolbar.addStretch()
        layout.addLayout(toolbar)
        layout.addStretch()

    def set_standard_view(self, name: str) -> None:
        angles = {
            "Side": (0.0, 0.0),
            "Front": (radians(90), 0.0),
            "Top": (0.0, radians(90)),
            "Isometric": (radians(-45), radians(30)),
        }
        if name not in angles:
            return
        self._yaw, self._pitch = angles[name]
        self.fit_view()

    def fit_view(self) -> None:
        self._projection_ready = False
        self._zoom = 1.0
        self.update()

    def set_tcp(self, tcp: Pose) -> None:
        """Use the session's tool transform for the TCP marker and axes."""
        self._model.tcp = tcp
        self.update()

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
        gripper: float = _OFFLINE_GRIPPER_POSITION,
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
        return self._model.stock_gripper_points(joints, gripper_position)

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
            self.height() * 0.50 + 10.0 - (z - center_z) * scale,
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
            max(1, self.height() - 130) / max(0.05, max_z - min_z),
        ) * 0.84
        self._projection_ready = True
        self._projection_size = (self.width(), self.height())

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
        segments = self._model.presentation_link_segments(joints)
        thicknesses = self._model.presentation_link_thicknesses()
        gripper = self._gripper_geometry_for(joints, gripper_position)

        style = Qt.PenStyle.DashLine if ghost else Qt.PenStyle.SolidLine
        scale = self._projection_scale * self._zoom
        for name, (first, second) in segments.items():
            body_width = _presentation_body_width(
                thicknesses[name], scale, ghost=ghost
            )
            painter.setPen(
                QPen(
                    link_color,
                    body_width,
                    style,
                    Qt.PenCapStyle.FlatCap,
                )
            )
            painter.drawLine(self._project(first), self._project(second))

        width = 2.8 if ghost else 4.0
        painter.setPen(QPen(link_color, width, style, Qt.PenCapStyle.FlatCap))
        for start, end in (("origin", "fixed_root"), ("origin", "moving_pivot")):
            painter.drawLine(self._project(gripper[start]), self._project(gripper[end]))

        finger_width = 2.2 if ghost else 3.8
        painter.setPen(QPen(link_color, finger_width, style, Qt.PenCapStyle.FlatCap))
        painter.drawLine(
            self._project(gripper["fixed_root"]),
            self._project(gripper["fixed_tip"]),
        )
        jaw_color = link_color
        if not ghost:
            jaw_color = (
                QColor("#17181b")
                if link_color.lightness() < 128
                else QColor("#e2e2e5")
            )
        jaw_width = 3.0 if ghost else 5.2
        painter.setPen(QPen(jaw_color, jaw_width, style, Qt.PenCapStyle.FlatCap))
        for start, end in (
            ("moving_pivot", "moving_root"),
            ("moving_root", "moving_tip"),
        ):
            painter.drawLine(self._project(gripper[start]), self._project(gripper[end]))

        painter.setBrush(QBrush(joint_color))
        painter.setPen(QPen(link_color, 1.0))
        radius = 2.2 if ghost else 3.2
        for point in joint_points:
            projected = self._project(point)
            painter.drawEllipse(projected, radius, radius)

        if label and ghost:
            tip = self._project(gripper["fixed_tip"])
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
        segments = self._model.presentation_link_segments(self._joints)
        tcp_pose = self._model.forward(self._joints)
        tcp = tcp_pose.position
        gripper = self.gripper_geometry()
        fit_points = [
            *points.values(),
            *(point for segment in segments.values() for point in segment),
            *gripper.values(),
            np.zeros(3, dtype=float),
            np.array([0.12, 0.0, 0.0]),
            np.array([0.0, 0.12, 0.0]),
            np.array([0.0, 0.0, 0.12]),
        ]

        if self._secondary_joints is not None:
            secondary_points = self._model.link_points(self._secondary_joints)
            secondary_segments = self._model.presentation_link_segments(
                self._secondary_joints
            )
            secondary_gripper = self._gripper_geometry_for(
                self._secondary_joints,
                self._secondary_gripper,
            )
            fit_points.extend(secondary_points.values())
            fit_points.extend(
                point
                for segment in secondary_segments.values()
                for point in segment
            )
            fit_points.extend(secondary_gripper.values())
        if self._target_position_m is not None:
            fit_points.append(self._target_position_m)
        if (
            not self._projection_ready
            or self.auto_fit.isChecked()
            or self._projection_size != (self.width(), self.height())
        ):
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
            painter.drawText(12, 56, f"{self._queue_depth} Cartesian jogs queued")

        # Screen-plane ruler: orthographic scale is uniform; world segments can
        # still be foreshortened when they point into the screen.
        ruler_mm = 50 if self._projection_scale * self._zoom * 0.05 < self.width() / 3 else 20
        length = ruler_mm / 1000.0 * self._projection_scale * self._zoom
        y = self.height() - 45
        painter.setPen(QPen(text, 1.3))
        painter.drawLine(QPointF(12, y), QPointF(12 + length, y))
        for x in (12, 12 + length):
            painter.drawLine(QPointF(x, y - 3), QPointF(x, y + 3))
        painter.drawText(QPointF(12, y - 6), f"{ruler_mm} mm")

        painter.setPen(QPen(_mix(muted, text, 0.20), 1.0))
        painter.drawText(
            12,
            self.height() - 11,
            "URDF visual schematic · Grid: model Z=0",
        )
        painter.end()

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self.view_combo.setCurrentText("Side")
            self.set_standard_view("Side")
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
            self.view_combo.setCurrentText("Free")
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
