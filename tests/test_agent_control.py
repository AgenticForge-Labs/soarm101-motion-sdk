from __future__ import annotations

import numpy as np
import pytest

from soarm101_motion.agent_control import (
    AgentAuthorityStore,
    evaluate_agent_jog,
)
from soarm101_motion.workspace import fit_paper_workspace


def _identity_workspace(*, calibration_id: str = "sha256:motor"):
    width = 0.2159
    height = 0.2794
    return fit_paper_workspace(
        {
            "A": (0.0, 0.0, 0.0),
            "B": (width, 0.0, 0.0),
            "C": (width, height, 0.0),
            "D": (0.0, height, 0.0),
        },
        (0.0, height, 0.050),
        robot_id="so101",
        arm_calibration_id=calibration_id,
        width_m=width,
        height_m=height,
        reference_height_m=0.050,
    )


def test_agent_authority_is_time_bounded_and_identity_bound(tmp_path) -> None:
    store = AgentAuthorityStore(tmp_path / "authority.json")
    authority = store.issue(
        robot_id="so101",
        calibration_id="sha256:motor",
        minutes=2.0,
        now=100.0,
    )

    assert authority.active(now=219.0)
    assert not authority.active(now=220.0)
    assert store.require(
        robot_id="so101",
        calibration_id="sha256:motor",
        now=150.0,
    ).robot_id == "so101"

    with pytest.raises(PermissionError, match="robot"):
        store.require(
            robot_id="other",
            calibration_id="sha256:motor",
            now=150.0,
        )
    with pytest.raises(PermissionError, match="calibration"):
        store.require(
            robot_id="so101",
            calibration_id="sha256:changed",
            now=150.0,
        )
    with pytest.raises(PermissionError, match="expired"):
        store.require(
            robot_id="so101",
            calibration_id="sha256:motor",
            now=221.0,
        )
    assert not store.path.exists()


def test_agent_jog_allows_50_mm_only_above_100_mm() -> None:
    workspace = _identity_workspace()
    decision = evaluate_agent_jog(
        workspace,
        active_calibration_id="sha256:motor",
        current_model_position_m=(0.1, 0.1, 0.101),
        delta_model_m=(0.03, 0.04, 0.0),
    )
    assert decision.maximum_distance_m == pytest.approx(0.050)
    assert decision.requested_distance_m == pytest.approx(0.050)

    with pytest.raises(PermissionError, match="50.0 mm limit"):
        evaluate_agent_jog(
            workspace,
            active_calibration_id="sha256:motor",
            current_model_position_m=(0.1, 0.1, 0.101),
            delta_model_m=(0.051, 0.0, 0.0),
        )


def test_agent_jog_tightens_to_10_mm_near_ground() -> None:
    workspace = _identity_workspace()
    decision = evaluate_agent_jog(
        workspace,
        active_calibration_id="sha256:motor",
        current_model_position_m=(0.1, 0.1, 0.080),
        delta_model_m=(0.006, 0.008, 0.0),
    )
    assert decision.maximum_distance_m == pytest.approx(0.010)
    assert decision.requested_distance_m == pytest.approx(0.010)

    with pytest.raises(PermissionError, match="10.0 mm limit"):
        evaluate_agent_jog(
            workspace,
            active_calibration_id="sha256:motor",
            current_model_position_m=(0.1, 0.1, 0.080),
            delta_model_m=(0.011, 0.0, 0.0),
        )


def test_agent_jog_rejects_crossing_calibrated_ground() -> None:
    workspace = _identity_workspace()
    with pytest.raises(PermissionError, match="ground plane"):
        evaluate_agent_jog(
            workspace,
            active_calibration_id="sha256:motor",
            current_model_position_m=(0.1, 0.1, 0.005),
            delta_model_m=(0.0, 0.0, -0.006),
        )


def test_agent_jog_requires_matching_workspace_calibration() -> None:
    workspace = _identity_workspace()
    with pytest.raises(PermissionError, match="does not match"):
        evaluate_agent_jog(
            workspace,
            active_calibration_id="sha256:other",
            current_model_position_m=np.array([0.1, 0.1, 0.1]),
            delta_model_m=np.array([0.001, 0.0, 0.0]),
        )
