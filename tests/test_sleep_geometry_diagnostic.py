from __future__ import annotations

import pytest

import soarm101_motion.safety as safety
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.exceptions import SafetyViolationError


def test_sleep_family_workspace_disables_only_self_clearance(monkeypatch) -> None:
    observed = {}

    def validate(model, path, *, tcp=None, **kwargs):
        del model, tcp
        observed["path"] = tuple(path)
        observed["kwargs"] = dict(kwargs)

    monkeypatch.setattr(safety, "validate_workspace_path", validate)

    samples = (
        {name: 0.0 for name in ARM_JOINTS},
        {name: 0.1 for name in ARM_JOINTS},
    )

    safety.validate_sleep_family_workspace_path(
        object(),
        samples,
        tcp=None,
        minimum_z_m=0.0,
        maximum_tcp_reach_m=0.50,
        minimum_self_clearance_m=0.025,
        base_keepout_radius_m=0.055,
        base_keepout_height_m=0.11,
    )

    assert observed["kwargs"]["minimum_self_clearance_m"] == 0.0
    assert observed["kwargs"]["minimum_z_m"] == 0.0
    assert observed["kwargs"]["maximum_tcp_reach_m"] == 0.50
    assert observed["kwargs"]["base_keepout_radius_m"] == 0.055
    assert observed["kwargs"]["base_keepout_height_m"] == 0.11
    assert observed["path"] == samples


def test_sleep_family_workspace_preserves_other_workspace_rejections(monkeypatch) -> None:
    def reject(*args, **kwargs):
        del args, kwargs
        raise SafetyViolationError("workspace check: wrist_flex enters base keep-out")

    monkeypatch.setattr(safety, "validate_workspace_path", reject)

    samples = (
        {name: 0.0 for name in ARM_JOINTS},
        {name: 0.1 for name in ARM_JOINTS},
    )

    with pytest.raises(SafetyViolationError, match="base keep-out"):
        safety.validate_sleep_family_workspace_path(
            object(),
            samples,
        )
