"""Simulation-only tests for the staged direct SDK broker executor."""

from __future__ import annotations

import pytest

from soarm101_motion.agent_control import AgentAuthorityStore
from soarm101_motion.broker import AgentMotionRates
from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.config import SOARM101Config


@pytest.fixture
def native(tmp_path):
    auth = AgentAuthorityStore(tmp_path / "human-lease.json")
    executor = SDKAgentExecutor(
        config=SOARM101Config(robot_id="so101"),
        rates=AgentMotionRates(),
        simulation=True,
        authority_store=auth,
    )
    yield executor, auth
    executor.close()


def test_hardware_direct_sdk_execution_is_not_available_by_default() -> None:
    with pytest.raises(ValueError, match="disabled pending supervised"):
        SDKAgentExecutor(
            config=SOARM101Config(robot_id="so101"),
            rates=AgentMotionRates(),
        )


def test_simulation_session_is_lazy_persistent_and_read_only_by_default(native) -> None:
    executor, auth = native
    assert executor._session is None
    capabilities = executor.execute("capabilities", {})
    assert "actions" in capabilities
    assert executor._session is None
    result = executor.execute("state", {})
    assert result["robot_id"] == "so101"
    assert result["calibration_id"] == "simulation"
    assert len(result["tcp_xyz_mm"]) == 3
    first = executor._session
    assert first is not None
    assert first.get_state().torque_enabled is False
    assert executor.execute("state", {})["robot_id"] == "so101"
    assert executor._session is first
    assert auth.status()["armed"] is False


def test_unknown_parameters_and_unsafe_actions_fail_before_session(native) -> None:
    executor, _ = native
    for action, request in (
        ("joint", {"joint": "shoulder_pan", "delta_deg": 1, "speed_deg_s": 100}),
        ("state", {"port": "/dev/ttyACM0"}),
        ("capture", {"camera": "overhead", "output": "/tmp/unsafe.jpg"}),
        ("raw_servos", {"value": [1, 2, 3]}),
        ("sleep", {"calibration": "overridden"}),
    ):
        with pytest.raises(ValueError):
            executor.execute(action, request)
    assert executor._session is None


def test_no_lease_cannot_enable_torque(native) -> None:
    executor, _ = native
    with pytest.raises(PermissionError, match="not armed"):
        executor.execute("joint", {"joint": "shoulder_pan", "delta_deg": 1.0})
    assert executor._session is not None
    assert executor._session.get_state().torque_enabled is False


def test_stop_does_not_implicitly_open_or_enable_robot(native) -> None:
    executor, _ = native
    with pytest.raises(RuntimeError, match="no connected SDK session"):
        executor.execute("stop", {})
    assert executor._session is None
    executor.execute("state", {})
    response = executor.execute("stop", {})
    assert response["action"] == "stop"
    assert response["completed"] is True
    assert executor._session.get_state().torque_enabled is False


def test_preview_rejects_physical_jog_without_measured_workspace(native) -> None:
    executor, auth = native
    auth.issue(robot_id="so101", calibration_id="simulation", minutes=1)
    with pytest.raises(PermissionError, match="measured workspace"):
        executor.execute(
            "jog", {"frame": "world", "x_mm": 1, "y_mm": 0, "z_mm": 0},
        )
    assert executor._session.get_state().torque_enabled is False


def test_expired_authority_refuses_motion(native) -> None:
    executor, auth = native
    auth.issue(robot_id="so101", calibration_id="simulation", minutes=1,
               now=1.0)
    with pytest.raises(PermissionError, match="expired"):
        executor.execute("joint", {"joint": "shoulder_pan", "delta_deg": 2.0})
    assert executor._session.get_state().torque_enabled is False


def test_invalid_joint_and_bool_delta_are_rejected(native) -> None:
    executor, auth = native
    auth.issue(robot_id="so101", calibration_id="simulation", minutes=1)
    with pytest.raises(ValueError, match="joint"):
        executor.execute("joint", {"joint": "motor_6", "delta_deg": 1.0})
    with pytest.raises(ValueError, match="finite number"):
        executor.execute("joint", {"joint": "shoulder_pan", "delta_deg": True})
    assert executor._session.get_state().torque_enabled is False
