from __future__ import annotations

from types import SimpleNamespace

import pytest

import examples.sleep_geometry_comparison as geometry
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.exceptions import SafetyViolationError


def _fake_arm() -> SimpleNamespace:
    return SimpleNamespace(
        config=SimpleNamespace(workspace_check_step_rad=0.05),
        model=object(),
        active_tcp=None,
        get_joint_limits=lambda: {name: (-2.0, 2.0) for name in ARM_JOINTS},
        _workspace_kwargs=lambda: {
            "minimum_z_m": 0.0,
            "maximum_tcp_reach_m": 0.50,
            "minimum_self_clearance_m": 0.025,
            "base_keepout_radius_m": 0.055,
            "base_keepout_height_m": 0.11,
        },
    )


def test_sleep_family_path_disables_only_self_clearance(monkeypatch) -> None:
    observed = {}

    def validate(model, path, *, tcp=None, **kwargs):
        del model, tcp
        observed["path"] = tuple(path)
        observed["kwargs"] = dict(kwargs)

    monkeypatch.setattr(geometry, "validate_workspace_path", validate)

    start = {name: 0.0 for name in ARM_JOINTS}
    target = dict(start)
    target["wrist_flex"] = 0.4

    geometry._validate_sleep_family_path(_fake_arm(), start, target)

    assert observed["kwargs"]["minimum_self_clearance_m"] == 0.0
    assert observed["kwargs"]["minimum_z_m"] == 0.0
    assert observed["kwargs"]["maximum_tcp_reach_m"] == 0.50
    assert observed["kwargs"]["base_keepout_radius_m"] == 0.055
    assert observed["kwargs"]["base_keepout_height_m"] == 0.11
    assert observed["path"][0] == start
    assert observed["path"][-1] == target


def test_sleep_family_path_preserves_other_workspace_rejections(monkeypatch) -> None:
    def reject(*args, **kwargs):
        del args, kwargs
        raise SafetyViolationError("workspace check: wrist_flex enters base keep-out")

    monkeypatch.setattr(geometry, "validate_workspace_path", reject)

    start = {name: 0.0 for name in ARM_JOINTS}
    target = dict(start)
    target["wrist_flex"] = 0.4

    with pytest.raises(SafetyViolationError, match="base keep-out"):
        geometry._validate_sleep_family_path(_fake_arm(), start, target)
