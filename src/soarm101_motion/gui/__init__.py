"""Optional PySide6 GUI for SO-ARM101 motion control.

Importing the GUI package must not require PySide6. Non-Qt helpers under
`soarm101_motion.gui` are used by software-only control logic and tests, while the
actual GUI remains an optional dependency loaded only when it is launched.
"""

from __future__ import annotations


def run_gui(argv: list[str] | None = None) -> int:
    """Launch the optional PySide6 GUI without importing Qt at package import time."""

    from soarm101_motion.gui.app import run_gui as _run_gui

    return _run_gui(argv)


__all__ = ["run_gui"]
