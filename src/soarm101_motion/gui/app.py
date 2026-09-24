"""Application entry point for the SO-ARM101 PySide6 controller."""

from __future__ import annotations

import argparse
import os
import sys

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from soarm101_motion.gui.window import MainWindow
from soarm101_motion.gui.session_log import configure


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="soarm101-gui",
        description="Control SO-ARM101 joints, Cartesian linear motion, and gripper.",
    )
    parser.add_argument("--port", help="serial port; selectable in the GUI")
    parser.add_argument("--robot-id", default="so101")
    parser.add_argument("--simulation", action="store_true", help="start in simulation mode")
    parser.add_argument("--no-session-log", action="store_true", help="disable GUI session logging")
    parser.add_argument(
        "--smoke-test",
        action="store_true",
        help="construct the GUI offscreen and exit; intended for CI",
    )
    return parser


def run_gui(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.smoke_test:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    app = QApplication.instance() or QApplication(sys.argv[:1])
    app.setApplicationName("SO-ARM101 Control")
    app.setOrganizationName("AgenticForge Labs")
    log_path = configure(enabled=not args.no_session_log and not args.smoke_test)
    window = MainWindow(
        port=args.port,
        robot_id=args.robot_id,
        simulation=args.simulation,
    )
    if log_path is not None:
        window._log(f"Session log: {log_path}")
    if args.smoke_test:
        window.show()
    else:
        window.showMaximized()
        if not args.simulation:
            QTimer.singleShot(0, window._find_arms)
    if args.smoke_test:
        QTimer.singleShot(250, window.close)
        QTimer.singleShot(1000, app.quit)
    return int(app.exec())


def main() -> int:
    return run_gui()


if __name__ == "__main__":
    raise SystemExit(main())
