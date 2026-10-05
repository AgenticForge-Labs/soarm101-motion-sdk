from __future__ import annotations

from types import SimpleNamespace

import pytest

import examples.sleep_geometry_comparison as geometry
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.exceptions import SafetyViolationError


def _fake_arm() -> SimpleNamespace:
    return SimpleNamespace(
        config=SimpleNamespace(
            minimum_self_clearance_m=0.025,
            workspace_check_step_rad=0.05,
        ),
        model=object(),
        active_tcp=None,
        _workspace_kwargs=lambda: {
            "minimum_z_m": 0.0,
            "maximum_tcp_reach_m": 0.50,
            "minimum_self_clearance_m": 0.025,
            "base_keepout_radius_m": 0.055,
            "base_keepout_height_m": 0.11,
        },
    )


def test_open_pre_sleep_selector_returns_deepest_passing_candidate(monkeypatch) -> None:
    start = {name: 0.0 for name in ARM_JOINTS}
    desired = {name: 1.0 for name in ARM_JOINTS}

    def validate(model, path, *, tcp=None, **kwargs):
        del model, tcp, kwargs
        if path[-1]["shoulder_pan"] > 0.60 + 1e-12:
            raise SafetyViolationError("too deep")

    monkeypatch.setattr(geometry, "validate_workspace_path", validate)
    monkeypatch.setattr(
        geometry,
        "minimum_workspace_self_clearance",
        lambda *args, **kwargs: 0.0275,
    )

    target, fraction, clearance = geometry._deepest_safe_open_target(
        _fake_arm(),
        start,
        desired,
    )

    assert fraction == pytest.approx(0.60)
    assert target["shoulder_pan"] == pytest.approx(0.60)
    assert clearance == pytest.approx(0.0275)


def test_open_pre_sleep_selector_does_not_hide_non_safety_errors(monkeypatch) -> None:
    start = {name: 0.0 for name in ARM_JOINTS}
    desired = {name: 1.0 for name in ARM_JOINTS}

    def fail(*args, **kwargs):
        del args, kwargs
        raise RuntimeError("diagnostic bug")

    monkeypatch.setattr(geometry, "validate_workspace_path", fail)

    with pytest.raises(RuntimeError, match="diagnostic bug"):
        geometry._deepest_safe_open_target(
            _fake_arm(),
            start,
            desired,
        )
