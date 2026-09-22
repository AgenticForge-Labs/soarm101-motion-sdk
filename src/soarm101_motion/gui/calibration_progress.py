"""Live circular indicators for mechanical-stop calibration sweep coverage."""

from __future__ import annotations

from typing import Mapping

from PySide6.QtCore import QRectF, Qt
from PySide6.QtGui import QPainter, QPen
from PySide6.QtWidgets import QGridLayout, QWidget

from soarm101_motion.constants import ALL_MOTORS


class SweepGauge(QWidget):
    """Compact pie-style progress indicator for one motor's observed sweep span."""

    def __init__(self, motor: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.motor = motor
        self.travel_ticks = 0
        self.required_ticks = 1
        self.fraction = 0.0
        self.passed = False
        self.setMinimumSize(150, 150)

    def set_progress(
        self,
        *,
        travel_ticks: int,
        required_ticks: int,
        fraction: float,
        passed: bool,
    ) -> None:
        self.travel_ticks = max(0, int(travel_ticks))
        self.required_ticks = max(1, int(required_ticks))
        self.fraction = max(0.0, min(1.0, float(fraction)))
        self.passed = bool(passed)
        self.update()

    def reset_progress(self) -> None:
        self.set_progress(
            travel_ticks=0,
            required_ticks=self.required_ticks,
            fraction=0.0,
            passed=False,
        )

    def paintEvent(self, _event: object) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        palette = self.palette()

        width = float(self.width())
        circle_size = min(width - 18.0, float(self.height()) - 58.0)
        circle_size = max(40.0, circle_size)
        left = (width - circle_size) / 2.0
        circle = QRectF(left, 6.0, circle_size, circle_size)

        painter.setPen(QPen(palette.mid().color(), 1.5))
        painter.setBrush(palette.base())
        painter.drawEllipse(circle)

        if self.fraction > 0.0:
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(palette.highlight())
            span = -round(360.0 * 16.0 * self.fraction)
            painter.drawPie(circle, 90 * 16, span)

        inner = circle.adjusted(
            circle.width() * 0.24,
            circle.height() * 0.24,
            -circle.width() * 0.24,
            -circle.height() * 0.24,
        )
        painter.setBrush(palette.window())
        painter.setPen(QPen(palette.mid().color(), 1.0))
        painter.drawEllipse(inner)

        painter.setPen(palette.text().color())
        percent = round(self.fraction * 100.0)
        painter.drawText(
            inner,
            Qt.AlignmentFlag.AlignCenter,
            f"{percent}%\n{'PASS' if self.passed else 'sweep'}",
        )

        title = self.motor.replace("_", " ").title()
        title_rect = QRectF(4.0, circle.bottom() + 5.0, width - 8.0, 20.0)
        painter.drawText(title_rect, Qt.AlignmentFlag.AlignCenter, title)
        tick_rect = QRectF(4.0, circle.bottom() + 24.0, width - 8.0, 20.0)
        painter.drawText(
            tick_rect,
            Qt.AlignmentFlag.AlignCenter,
            f"{self.travel_ticks} / {self.required_ticks} ticks",
        )


class CalibrationSweepPanel(QWidget):
    """Six live sweep gauges arranged as a compact calibration dashboard."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QGridLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.gauges: dict[str, SweepGauge] = {}
        for index, motor in enumerate(ALL_MOTORS):
            gauge = SweepGauge(motor)
            self.gauges[motor] = gauge
            layout.addWidget(gauge, index // 3, index % 3)

    def set_progress(self, progress: Mapping[str, Mapping[str, object]]) -> None:
        for motor, gauge in self.gauges.items():
            values = progress.get(motor)
            if values is None:
                continue
            gauge.set_progress(
                travel_ticks=int(values.get("travel_ticks", 0)),
                required_ticks=int(values.get("required_ticks", 1)),
                fraction=float(values.get("fraction", 0.0)),
                passed=bool(values.get("passed", False)),
            )

    def reset(self, required_ticks: Mapping[str, int] | None = None) -> None:
        for motor, gauge in self.gauges.items():
            if required_ticks is not None and motor in required_ticks:
                gauge.required_ticks = max(1, int(required_ticks[motor]))
            gauge.reset_progress()
