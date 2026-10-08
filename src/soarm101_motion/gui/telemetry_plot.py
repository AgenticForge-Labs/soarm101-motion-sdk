"""Live teleoperation joint trace using measurements already owned by the GUI workers."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from time import perf_counter
from typing import Mapping

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPalette, QPen, QPolygonF
from PySide6.QtWidgets import QWidget

from soarm101_motion.constants import ARM_JOINTS


@dataclass(frozen=True)
class _TraceSample:
    timestamp: float
    follower_deg: dict[str, float]
    leader_deg: dict[str, float] | None


class TeleopMotionTrace(QWidget):
    """Scrolling five-joint trace for teleoperation diagnostics.

    The widget consumes leader stream samples and follower measurements that already
    exist for teleoperation. It performs no motor/register polling of its own.
    """

    HISTORY_SECONDS = 10.0
    MAX_SAMPLES = 800

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._samples: deque[_TraceSample] = deque(maxlen=self.MAX_SAMPLES)
        self._latest_leader_deg: dict[str, float] | None = None
        self._mode = "angles"
        self.setMinimumHeight(185)
        self.setToolTip(
            "Live measured teleoperation history. Angles overlays follower (solid) and "
            "leader (dashed); tracking error shows follower minus leader. This view adds "
            "no extra motor reads."
        )

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    def clear(self) -> None:
        self._samples.clear()
        self._latest_leader_deg = None
        self.update()

    def set_mode(self, mode: str) -> None:
        mode = str(mode)
        if mode not in {"angles", "error"}:
            raise ValueError(f"unsupported teleop trace mode: {mode}")
        self._mode = mode
        self.update()

    def set_leader_sample(
        self,
        joints_deg: Mapping[str, float],
        *,
        timestamp: float | None = None,
    ) -> None:
        del timestamp
        self._latest_leader_deg = {
            name: float(joints_deg[name]) for name in ARM_JOINTS
        }

    def add_follower_sample(
        self,
        joints_deg: Mapping[str, float],
        *,
        timestamp: float | None = None,
    ) -> None:
        now = perf_counter() if timestamp is None else float(timestamp)
        self._samples.append(
            _TraceSample(
                timestamp=now,
                follower_deg={
                    name: float(joints_deg[name]) for name in ARM_JOINTS
                },
                leader_deg=(
                    None
                    if self._latest_leader_deg is None
                    else dict(self._latest_leader_deg)
                ),
            )
        )
        cutoff = now - self.HISTORY_SECONDS
        while self._samples and self._samples[0].timestamp < cutoff:
            self._samples.popleft()
        self.update()

    @staticmethod
    def _series_color(palette: QPalette, index: int) -> QColor:
        base = palette.color(QPalette.ColorRole.Highlight)
        hue = base.hue()
        if hue < 0:
            hue = 20
        saturation = max(120, base.saturation())
        value = max(170, base.value())
        return QColor.fromHsv((hue + index * 58) % 360, saturation, value)

    def paintEvent(self, _event: object) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        palette = self.palette()
        text = palette.color(QPalette.ColorRole.Text)
        muted = palette.color(QPalette.ColorRole.Mid)
        base = palette.color(QPalette.ColorRole.Base)

        outer = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        painter.setPen(QPen(muted, 1.0))
        painter.setBrush(base)
        painter.drawRoundedRect(outer, 9.0, 9.0)

        if not self._samples:
            painter.setPen(text)
            painter.drawText(
                outer,
                Qt.AlignmentFlag.AlignCenter,
                "Start teleoperation to see measured joint motion.",
            )
            return

        left = 96.0
        right = max(left + 20.0, float(self.width()) - 10.0)
        top = 24.0
        bottom = max(top + 20.0, float(self.height()) - 8.0)
        row_height = (bottom - top) / len(ARM_JOINTS)
        latest = self._samples[-1].timestamp
        window_start = latest - self.HISTORY_SECONDS

        painter.setPen(QPen(text, 1.0))
        legend = (
            "Follower solid · leader dashed · 10 s"
            if self._mode == "angles"
            else "Follower − leader tracking error · 10 s"
        )
        painter.drawText(QPointF(left, 16.0), legend)

        def x_for(timestamp: float) -> float:
            fraction = (timestamp - window_start) / self.HISTORY_SECONDS
            return left + max(0.0, min(1.0, fraction)) * (right - left)

        for index, name in enumerate(ARM_JOINTS):
            row_top = top + index * row_height
            row_bottom = row_top + row_height
            row_mid = (row_top + row_bottom) / 2.0
            painter.setPen(QPen(muted, 0.7))
            painter.drawLine(QPointF(left, row_bottom), QPointF(right, row_bottom))
            painter.setPen(QPen(text, 1.0))
            painter.drawText(
                QRectF(7.0, row_top, left - 14.0, row_height),
                Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft,
                name.replace("_", " "),
            )

            color = self._series_color(palette, index)
            if self._mode == "error":
                values = [
                    sample.follower_deg[name] - sample.leader_deg[name]
                    for sample in self._samples
                    if sample.leader_deg is not None
                ]
                if not values:
                    continue
                bound = max(0.5, max(abs(value) for value in values) * 1.15)
                painter.setPen(QPen(muted, 0.7, Qt.PenStyle.DashLine))
                painter.drawLine(QPointF(left, row_mid), QPointF(right, row_mid))

                def y_error(value: float) -> float:
                    return row_mid - (value / bound) * (row_height * 0.42)

                points = [
                    QPointF(
                        x_for(sample.timestamp),
                        y_error(sample.follower_deg[name] - sample.leader_deg[name]),
                    )
                    for sample in self._samples
                    if sample.leader_deg is not None
                ]
                if len(points) >= 2:
                    painter.setPen(QPen(color, 1.7))
                    painter.drawPolyline(QPolygonF(points))
                continue

            follower_values = [sample.follower_deg[name] for sample in self._samples]
            leader_values = [
                sample.leader_deg[name]
                for sample in self._samples
                if sample.leader_deg is not None
            ]
            combined = follower_values + leader_values
            low = min(combined)
            high = max(combined)
            span = max(8.0, high - low)
            center = (low + high) / 2.0
            low = center - span * 0.60
            high = center + span * 0.60

            def y_angle(value: float) -> float:
                fraction = (value - low) / max(1e-6, high - low)
                return row_bottom - 3.0 - fraction * max(1.0, row_height - 6.0)

            follower_points = [
                QPointF(x_for(sample.timestamp), y_angle(sample.follower_deg[name]))
                for sample in self._samples
            ]
            if len(follower_points) >= 2:
                painter.setPen(QPen(color, 1.8))
                painter.drawPolyline(QPolygonF(follower_points))

            leader_points = [
                QPointF(x_for(sample.timestamp), y_angle(sample.leader_deg[name]))
                for sample in self._samples
                if sample.leader_deg is not None
            ]
            if len(leader_points) >= 2:
                painter.setPen(QPen(color, 1.2, Qt.PenStyle.DashLine))
                painter.drawPolyline(QPolygonF(leader_points))
