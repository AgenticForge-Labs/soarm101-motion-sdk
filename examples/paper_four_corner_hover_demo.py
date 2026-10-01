"""Compatibility wrapper for the renamed paper workspace calibration workflow.

The historical four-corner hover demo no longer performs autonomous Cartesian motion.
Use examples/paper_workspace_calibration.py for the current manual table/workspace
measurement workflow.
"""

from __future__ import annotations

from paper_workspace_calibration import main


if __name__ == "__main__":
    raise SystemExit(main())
