"""Offscreen Qt event-loop benchmark for the mesh viewer and status sidebar.

This is a software-only benchmark. It never discovers or connects to hardware.
"""

from __future__ import annotations

import argparse
import os
import statistics
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QEvent, QObject, QPoint, Qt, QTimer
from PySide6.QtGui import QImage, QPainter
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from soarm101_motion.constants import ARM_JOINTS, HOME_JOINTS
from soarm101_motion.gui.arm_status import RobotStatusPanel
from soarm101_motion.gui.cartesian_view import CartesianArmView


def render(view: CartesianArmView) -> None:
    """Time the legacy synchronous paint path separately from event-loop work."""
    image = QImage(view.size(), QImage.Format.Format_ARGB32_Premultiplied)
    painter = QPainter(image)
    view.render(painter, QPoint())
    painter.end()


class PaintTimer(QObject):
    """Collect queued paint dispatch latency and periodic timer lateness."""

    def __init__(self, app: QApplication, interval_ms: int = 10) -> None:
        super().__init__(app)
        self.paint_ms: list[float] = []
        self.timer_late_ms: list[float] = []
        self._update_queued: float | None = None
        self._interval = interval_ms / 1000
        self._expected = 0.0
        self.timer = QTimer(self)
        self.timer.setInterval(interval_ms)
        self.timer.timeout.connect(self._heartbeat)
        app.installEventFilter(self)

    def start(self) -> None:
        self._expected = time.perf_counter() + self._interval
        self.timer.start()

    def stop(self) -> None:
        self.timer.stop()

    def _heartbeat(self) -> None:
        now = time.perf_counter()
        self.timer_late_ms.append(max(0.0, now - self._expected) * 1000)
        self._expected = now + self._interval

    def eventFilter(self, watched: QObject, event: QEvent) -> bool:
        if event.type() == QEvent.Type.UpdateRequest:
            self._update_queued = time.perf_counter()
        elif event.type() == QEvent.Type.Paint and self._update_queued is not None:
            self.paint_ms.append((time.perf_counter() - self._update_queued) * 1000)
            self._update_queued = None
        return False


def report(label: str, values: list[float], unit: str = "ms") -> None:
    if not values:
        print(f"{label}: no samples")
        return
    print(
        f"{label}: n={len(values)} median={statistics.median(values):.2f}{unit} "
        f"slowest={max(values):.2f}{unit}"
    )


def drain(app: QApplication) -> None:
    app.processEvents()
    QTest.qWait(1)
    app.processEvents()


def benchmark_view(view: CartesianArmView, app: QApplication, runs: int) -> None:
    timer = PaintTimer(app)
    timer.start()
    drain(app)
    # Apples-to-apples synchronous paints are retained but clearly separated.
    direct: list[float] = []
    for _ in range(runs):
        started = time.perf_counter()
        render(view)
        direct.append((time.perf_counter() - started) * 1000)
    report("direct_render_static", direct)

    # Drag uses actual Qt press/move/release events; each move is dispatched via
    # the widget's event machinery and queued paints are serviced by Qt.
    drag_ms: list[float] = []
    center = view.rect().center()
    QTest.mousePress(view, Qt.MouseButton.LeftButton, pos=center)
    for index in range(runs):
        start = time.perf_counter()
        QTest.mouseMove(view, center + QPoint(8 + index % 4, 5), delay=1)
        drain(app)
        drag_ms.append((time.perf_counter() - start) * 1000)
    QTest.mouseRelease(view, Qt.MouseButton.LeftButton, pos=center + QPoint(8, 5))
    drain(app)
    report("qt_continuous_drag_dispatch_and_paint", drag_ms)

    joints = dict(HOME_JOINTS)
    same = {name: float(joints[name]) for name in ARM_JOINTS}
    view.set_joint_degrees(same)
    view.set_gripper_position(0.5)
    drain(app)
    repeated_ms: list[float] = []
    for _ in range(runs):
        start = time.perf_counter()
        view.set_joint_degrees(same)
        view.set_gripper_position(0.5)
        drain(app)
        repeated_ms.append((time.perf_counter() - start) * 1000)
    report("identical_measured_joint_and_jaw_update", repeated_ms)

    changing_ms: list[float] = []
    for index in range(runs):
        joints[ARM_JOINTS[index % len(ARM_JOINTS)]] += 0.15 if index % 2 == 0 else -0.15
        start = time.perf_counter()
        view.set_joint_degrees(joints)
        view.set_gripper_position(0.45 + 0.01 * (index % 5))
        drain(app)
        changing_ms.append((time.perf_counter() - start) * 1000)
    report("changing_measured_joint_and_jaw_update", changing_ms)
    timer.stop()
    report("qt_queued_update_to_paint_dispatch_latency", timer.paint_ms)
    report("qt_timer_heartbeat_lateness", timer.timer_late_ms)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--size", type=int, default=480, help="square standalone view size")
    parser.add_argument("--runs", type=int, default=21)
    args = parser.parse_args()
    if args.size < 220 or args.runs < 3:
        parser.error("--size must be >= 220 and --runs must be >= 3")

    app = QApplication.instance() or QApplication([])
    view = CartesianArmView()
    view.resize(args.size, args.size)
    started = time.perf_counter()
    view.show()
    drain(app)
    print(f"standalone_viewport={view.width()}x{view.height()} cold_show_s={time.perf_counter()-started:.3f}")
    benchmark_view(view, app, args.runs)
    view.close()

    panel = RobotStatusPanel(compact=True)
    panel.resize(340, 900)
    panel.show()
    drain(app)
    panel_view = panel.view
    print(f"status_panel={panel.width()}x{panel.height()} view={panel_view.width()}x{panel_view.height()}")
    benchmark_view(panel_view, app, args.runs)
    panel.close()
    app.quit()


if __name__ == "__main__":
    main()
