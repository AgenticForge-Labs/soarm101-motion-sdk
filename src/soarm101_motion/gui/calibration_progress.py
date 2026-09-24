"""Live circular indicators for mechanical-stop calibration sweep coverage."""

from __future__ import annotations

from typing import Mapping

from PySide6.QtCore import QRectF, Qt, QTimer
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QGridLayout, QWidget

from soarm101_motion.constants import ALL_MOTORS
from soarm101_motion.calibration_live import SWEEP_DISPLAY_TRAVEL_TICKS


def sweep_color(fraction: float) -> QColor:
    """Move continuously from red through amber to green as coverage grows."""
    return QColor.fromHsvF(max(0.0, min(1.0, fraction)) / 3.0, 0.82, 0.86)


class SweepGauge(QWidget):
    """Two concentric progress rings, one per end-to-end traversal."""

    def __init__(self, motor: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.motor = motor
        self.travel_ticks = 0
        self.best_leg_ticks = 0
        self.required_ticks = 1
        self.display_ticks = 1
        self.fraction = 0.0
        self.passed = False
        self.sweep_number = 1
        self.traversals_completed = 0
        self.pop_progress: float | None = None
        self._pop_timer = QTimer(self)
        self._pop_timer.setInterval(16)
        self._pop_timer.timeout.connect(self._advance_pop)
        self.setMinimumSize(125, 150)
        self.setToolTip(
            "Inner ring: first full traversal. Outer ring: second. "
            "The best partial attempt stays visible, but only a full stop-to-stop "
            "traversal counts toward 2/2."
        )

    def set_progress(
        self,
        *,
        travel_ticks: int,
        best_leg_ticks: int | None = None,
        required_ticks: int,
        display_ticks: int | None = None,
        fraction: float,
        passed: bool,
        sweep_number: int = 1,
        traversals_completed: int = 0,
    ) -> None:
        just_completed = bool(passed) and not self.passed
        self.travel_ticks = max(0, int(travel_ticks))
        self.best_leg_ticks = max(0, int(
            travel_ticks if best_leg_ticks is None else best_leg_ticks
        ))
        self.required_ticks = max(1, int(required_ticks))
        self.display_ticks = max(self.required_ticks, int(display_ticks or required_ticks))
        self.fraction = max(0.0, min(1.0, float(fraction)))
        self.passed = bool(passed)
        self.sweep_number = max(1, min(2, int(sweep_number)))
        self.traversals_completed = max(0, min(2, int(traversals_completed)))
        if just_completed:
            self.pop_progress = 0.0
            self._pop_timer.start()
        self.update()

    def _advance_pop(self) -> None:
        if self.pop_progress is None:
            self._pop_timer.stop()
            return
        self.pop_progress += 0.08
        if self.pop_progress >= 1.0:
            self.pop_progress = None
            self._pop_timer.stop()
        self.update()

    def reset_progress(self) -> None:
        self._pop_timer.stop()
        self.pop_progress = None
        self.set_progress(
            travel_ticks=0,
            best_leg_ticks=0,
            required_ticks=self.required_ticks,
            display_ticks=self.display_ticks,
            fraction=0.0,
            passed=False,
            sweep_number=1,
            traversals_completed=0,
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

        first_fraction = 1.0 if self.sweep_number == 2 or self.passed else self.fraction
        second_fraction = self.fraction if self.sweep_number == 2 else 0.0
        if self.passed:
            second_fraction = 1.0

        ring_width = max(6.0, circle.width() * 0.105)
        ring_gap = max(5.0, circle.width() * 0.085)
        outer = circle.adjusted(
            ring_width / 2, ring_width / 2, -ring_width / 2, -ring_width / 2,
        )
        inset = ring_width + ring_gap
        inner = outer.adjusted(inset, inset, -inset, -inset)
        for rect, fraction in ((inner, first_fraction), (outer, second_fraction)):
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.setPen(QPen(palette.mid().color(), ring_width))
            painter.drawEllipse(rect)
            if fraction > 0.0:
                painter.setPen(QPen(sweep_color(fraction), ring_width, Qt.PenStyle.SolidLine,
                                    Qt.PenCapStyle.RoundCap))
                painter.drawArc(rect, 90 * 16, -round(360.0 * 16.0 * fraction))

        painter.setPen(palette.text().color())
        percent = round(self.fraction * 100.0)
        painter.drawText(
            inner.adjusted(ring_width / 2, ring_width / 2, -ring_width / 2, -ring_width / 2),
            Qt.AlignmentFlag.AlignCenter,
            f"{percent}%\n{'DONE' if self.passed else f'{self.sweep_number}/2'}",
        )

        if self.pop_progress is not None:
            progress = self.pop_progress
            burst = QColor(0, 0, 0, round(240 * (1.0 - progress)))
            painter.setPen(QPen(burst, max(1.0, 3.5 * (1.0 - progress))))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            radius = 4.0 + 23.0 * progress
            center = circle.center()
            painter.drawEllipse(center, radius, radius)

        title = self.motor.replace("_", " ").title()
        title_rect = QRectF(4.0, circle.bottom() + 5.0, width - 8.0, 20.0)
        painter.drawText(title_rect, Qt.AlignmentFlag.AlignCenter, title)
        tick_rect = QRectF(4.0, circle.bottom() + 24.0, width - 8.0, 20.0)
        painter.drawText(
            tick_rect,
            Qt.AlignmentFlag.AlignCenter,
            f"Best {self.best_leg_ticks} / {self.display_ticks}",
        )


class CalibrationSweepPanel(QWidget):
    """Six live sweep gauges arranged in one visible row."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QGridLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.gauges: dict[str, SweepGauge] = {}
        for index, motor in enumerate(ALL_MOTORS):
            gauge = SweepGauge(motor)
            self.gauges[motor] = gauge
            layout.addWidget(gauge, 0, index)

    def set_progress(self, progress: Mapping[str, Mapping[str, object]]) -> None:
        for motor, gauge in self.gauges.items():
            values = progress.get(motor)
            if values is None:
                continue
            gauge.set_progress(
                travel_ticks=int(values.get("travel_ticks", 0)),
                best_leg_ticks=int(values.get("best_leg_ticks", values.get("travel_ticks", 0))),
                required_ticks=int(values.get("required_ticks", 1)),
                display_ticks=int(values.get("display_ticks", values.get("required_ticks", 1))),
                fraction=float(values.get("fraction", 0.0)),
                passed=bool(values.get("passed", False)),
                sweep_number=int(values.get("sweep_number", 1)),
                traversals_completed=int(values.get("traversals_completed", 0)),
            )

    def reset(
        self,
        required_ticks: Mapping[str, int] | None = None,
        display_ticks: Mapping[str, int] | None = None,
    ) -> None:
        for motor, gauge in self.gauges.items():
            if required_ticks is not None and motor in required_ticks:
                gauge.required_ticks = max(1, int(required_ticks[motor]))
                gauge.display_ticks = max(
                    gauge.required_ticks,
                    (display_ticks or SWEEP_DISPLAY_TRAVEL_TICKS)[motor],
                )
            gauge.reset_progress()
