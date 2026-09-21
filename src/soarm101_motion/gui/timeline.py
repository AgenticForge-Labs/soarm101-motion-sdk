"""Lightweight PySide6 trajectory timeline for recorded SO-ARM101 motion."""

from __future__ import annotations

from math import isfinite

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QWidget

from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.trajectories import Trajectory


class TrajectoryTimeline(QWidget):
    """Paint joint/gripper curves plus crop selection and scrub cursor."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._trajectory: Trajectory | None = None
        self._selection = (0.0, 0.0)
        self._cursor_s = 0.0
        self.setMinimumHeight(300)

    def set_trajectory(self, trajectory: Trajectory | None) -> None:
        self._trajectory = trajectory
        duration = 0.0 if trajectory is None else trajectory.duration_s
        self._selection = (0.0, duration)
        self._cursor_s = 0.0
        self.update()

    def set_selection(self, start_s: float, end_s: float) -> None:
        trajectory = self._trajectory
        if trajectory is None:
            return
        start = max(0.0, min(float(start_s), trajectory.duration_s))
        end = max(start, min(float(end_s), trajectory.duration_s))
        self._selection = (start, end)
        self.update()

    def set_cursor(self, seconds: float) -> None:
        trajectory = self._trajectory
        if trajectory is None:
            return
        value = float(seconds)
        if not isfinite(value):
            return
        self._cursor_s = max(0.0, min(value, trajectory.duration_s))
        self.update()

    @staticmethod
    def _channel_path(
        painter_path: QPainterPath,
        times: np.ndarray,
        values: np.ndarray,
        rect: QRectF,
        duration: float,
    ) -> None:
        minimum = float(np.min(values))
        maximum = float(np.max(values))
        span = maximum - minimum
        if span < 1e-12:
            span = 1.0
        for index, (time_s, value) in enumerate(zip(times, values, strict=True)):
            x = rect.left() + (float(time_s) / duration) * rect.width()
            normalized = (float(value) - minimum) / span
            y = rect.bottom() - normalized * rect.height()
            point = QPointF(x, y)
            if index == 0:
                painter_path.moveTo(point)
            else:
                painter_path.lineTo(point)

    def paintEvent(self, _event: object) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        palette = self.palette()
        painter.fillRect(self.rect(), palette.base())

        trajectory = self._trajectory
        if trajectory is None or trajectory.duration_s <= 0:
            painter.setPen(palette.text().color())
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "No trajectory loaded")
            return

        margin_left = 105.0
        margin_right = 12.0
        margin_top = 16.0
        margin_bottom = 22.0
        plot = QRectF(
            margin_left,
            margin_top,
            max(1.0, self.width() - margin_left - margin_right),
            max(1.0, self.height() - margin_top - margin_bottom),
        )
        labels = [name.replace("_", " ").title() for name in ARM_JOINTS] + ["Gripper"]
        channels = [
            trajectory.joints_rad[:, index] for index in range(len(ARM_JOINTS))
        ] + [trajectory.gripper]
        lane_height = plot.height() / len(channels)
        grid_pen = QPen(palette.mid().color())
        curve_pen = QPen(palette.highlight().color(), 1.6)
        text_pen = QPen(palette.text().color())

        for index, (label, values) in enumerate(zip(labels, channels, strict=True)):
            lane = QRectF(
                plot.left(),
                plot.top() + index * lane_height,
                plot.width(),
                lane_height,
            )
            painter.setPen(grid_pen)
            painter.drawLine(lane.bottomLeft(), lane.bottomRight())
            painter.setPen(text_pen)
            painter.drawText(
                QRectF(4.0, lane.top(), margin_left - 10.0, lane.height()),
                Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight,
                label,
            )
            path = QPainterPath()
            self._channel_path(
                path,
                trajectory.timestamps_s,
                np.asarray(values),
                lane.adjusted(0.0, 4.0, 0.0, -4.0),
                trajectory.duration_s,
            )
            painter.setPen(curve_pen)
            painter.drawPath(path)

        start_s, end_s = self._selection
        start_x = plot.left() + start_s / trajectory.duration_s * plot.width()
        end_x = plot.left() + end_s / trajectory.duration_s * plot.width()
        outside = palette.window().color()
        outside.setAlpha(150)
        painter.fillRect(
            QRectF(plot.left(), plot.top(), start_x - plot.left(), plot.height()), outside
        )
        painter.fillRect(
            QRectF(end_x, plot.top(), plot.right() - end_x, plot.height()), outside
        )

        selection_pen = QPen(palette.highlight().color(), 2.0)
        painter.setPen(selection_pen)
        painter.drawLine(QPointF(start_x, plot.top()), QPointF(start_x, plot.bottom()))
        painter.drawLine(QPointF(end_x, plot.top()), QPointF(end_x, plot.bottom()))

        cursor_x = plot.left() + self._cursor_s / trajectory.duration_s * plot.width()
        cursor_pen = QPen(palette.text().color(), 1.0, Qt.PenStyle.DashLine)
        painter.setPen(cursor_pen)
        painter.drawLine(QPointF(cursor_x, plot.top()), QPointF(cursor_x, plot.bottom()))

        painter.setPen(text_pen)
        painter.drawText(
            QRectF(plot.left(), plot.bottom() + 2.0, plot.width(), margin_bottom - 2.0),
            Qt.AlignmentFlag.AlignLeft,
            "0.00 s",
        )
        painter.drawText(
            QRectF(plot.left(), plot.bottom() + 2.0, plot.width(), margin_bottom - 2.0),
            Qt.AlignmentFlag.AlignRight,
            f"{trajectory.duration_s:.2f} s",
        )
