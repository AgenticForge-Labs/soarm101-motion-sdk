"""Hardware-free tests of the trusted operator startup latch and fencing."""
from types import SimpleNamespace

import pytest

from soarm101_motion.broker import AgentMotionRates
from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.config import SOARM101Config
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.exceptions import HardwareFaultError, RobotConnectionError
import soarm101_motion.broker_sdk as broker_sdk


def fixture(monkeypatch, *, holding=False):
    executor = SDKAgentExecutor(config=SOARM101Config(robot_id="arm", port="/dev/unused",
        auto_enable_torque=False, configure_motors_on_connect=False, disable_torque_on_disconnect=False),
        rates=AgentMotionRates(), physical_actions=True, operator_choreography_only=True)
    events = []
    state = SimpleNamespace(connected=True, faulted=False, moving=False, torque_enabled=holding)
    positions = dict.fromkeys(ARM_JOINTS, 0.0)
    def enable():
        events.append("enable")
        assert executor._motion_action_attempted
        assert "positions" in events
        state.torque_enabled = True
    def read():
        events.append("positions")
        return SimpleNamespace(positions=dict(positions))
    arm = SimpleNamespace(config=SimpleNamespace(robot_id="arm"), calibration_id="cal-a",
        is_connected=True, get_state=lambda: state, get_joint_positions=read, enable=enable)
    executor._session = arm
    executor._bound_calibration_id = "cal-a"
    executor._lifecycle = "open"
    def check():
        executor.validate_supervision()
        events.append("watchdog")
    watchdog = SimpleNamespace(check=check)
    monkeypatch.setattr(broker_sdk, "STOP_STABILITY_SAMPLE_DELAYS_S", (0, 0))
    return executor, arm, state, positions, watchdog, events


def test_startup_latches_once_before_claiming_hold(monkeypatch):
    executor, arm, state, positions, watchdog, events = fixture(monkeypatch)
    result = executor.establish_operator_startup_hold(watchdog)
    assert result["holding"] is True and result["torque_enabled_at_startup"] is True
    assert events.count("enable") == 1
    assert executor._startup_hold_verified
    with pytest.raises(PermissionError, match="already attempted"):
        executor.establish_operator_startup_hold(watchdog)
    assert events.count("enable") == 1


def test_existing_hold_is_verified_without_redundant_latch(monkeypatch):
    executor, arm, state, positions, watchdog, events = fixture(monkeypatch, holding=True)
    assert executor.establish_operator_startup_hold(watchdog)["torque_enabled_at_startup"] is False
    assert "enable" not in events
    assert not executor._motion_action_attempted


@pytest.mark.parametrize("bad_state", ["moving", "faulted", "disconnected", "calibration", "watchdog"])
def test_readiness_rejection_cannot_enable_torque(monkeypatch, bad_state):
    executor, arm, state, positions, watchdog, events = fixture(monkeypatch)
    if bad_state == "disconnected":
        state.connected = False
    elif bad_state == "calibration":
        arm.calibration_id = "other"
    elif bad_state == "watchdog":
        def denied():
            raise PermissionError("watchdog terminal")
        watchdog.check = denied
    else:
        setattr(state, bad_state, True)
    with pytest.raises((PermissionError, RuntimeError, RobotConnectionError)):
        executor.establish_operator_startup_hold(watchdog)
    assert "enable" not in events


@pytest.mark.parametrize("failure", ["enable", "drift", "torque"])
def test_partial_or_uncertain_latch_is_terminal_without_retry(monkeypatch, failure):
    executor, arm, state, positions, watchdog, events = fixture(monkeypatch)
    original = arm.enable
    def fail():
        original()
        if failure == "enable":
            raise RuntimeError("lost enable receipt")
        if failure == "drift":
            positions[ARM_JOINTS[0]] = .1
        if failure == "torque":
            state.torque_enabled = False
    arm.enable = fail
    with pytest.raises((RuntimeError, HardwareFaultError)):
        executor.establish_operator_startup_hold(watchdog)
    assert executor.session_evidence["lifecycle"] == "faulted"
    with pytest.raises(RuntimeError, match="faulted"):
        executor.establish_operator_startup_hold(watchdog)
    assert events.count("enable") == 1


def test_agent_action_interface_does_not_expose_startup_hold(monkeypatch):
    executor, *_ = fixture(monkeypatch)
    with pytest.raises(ValueError, match="unknown SDK broker action"):
        executor.execute("establish_operator_startup_hold", {})
