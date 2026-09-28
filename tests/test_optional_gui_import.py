"""The optional GUI package must not make base SDK imports depend on PySide6."""

from __future__ import annotations

import subprocess
import sys


def test_gui_helpers_import_without_pyside6() -> None:
    code = r"""
import builtins

real_import = builtins.__import__

def blocked_import(name, *args, **kwargs):
    if name == "PySide6" or name.startswith("PySide6."):
        raise ModuleNotFoundError("PySide6 intentionally blocked for optional-dependency test")
    return real_import(name, *args, **kwargs)

builtins.__import__ = blocked_import

import soarm101_motion.gui
from soarm101_motion.gui.teleop_rate import limit_joint_target

assert callable(soarm101_motion.gui.run_gui)
assert callable(limit_joint_target)
"""
    subprocess.run([sys.executable, "-c", code], check=True)
