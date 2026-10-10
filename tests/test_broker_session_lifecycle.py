"""Hardware-free ownership and lifecycle regressions for the staged SDK broker.

Never construct a real Feetech backend or enable robot torque in these tests.
"""
from __future__ import annotations

import threading
import time
from types import SimpleNamespace

import pytest

from soarm101_motion.agent_control import AgentAuthorityStore
from soarm101_motion.broker import AgentMotionRates
from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.config import SOARM101Config
from soarm101_motion.exceptions import CommunicationError, RobotConnectionError


class FakeConnectedArm:
    def __init__(self, config):
        self.config = config
        self.calibration_id = "cal-good"
        self.is_connected = False
        self.connect_calls = 0
        self.disconnect_calls = 0
        self.stop_calls = 0

    def connect(self):
        self.connect_calls += 1
        self.is_connected = True

    def disconnect(self):
        self.disconnect_calls += 1
        self.is_connected = False

    def stop(self):
        self.stop_calls += 1

    def get_joint_positions(self):
        return SimpleNamespace(positions={"shoulder_pan": 0.0})

    def get_state(self):
        return SimpleNamespace(torque_enabled=True)


def make_executor(tmp_path, arm, *, physical=False):
    config = arm.config
    auth = AgentAuthorityStore(tmp_path / "lease.json")
    sdk = SDKAgentExecutor(
        config=config, rates=AgentMotionRates(), authority_store=auth,
        simulation=not physical, physical_trial=physical, arm_factory=lambda: arm,
    )
    return sdk, auth


def test_closed_session_never_implicitly_reconnects(tmp_path) -> None:
    arm = FakeConnectedArm(SOARM101Config(robot_id="so101"))
    sdk, _ = make_executor(tmp_path, arm)
    sdk._arm()
    assert sdk.session_evidence["lifecycle"] == "open"
    assert sdk.connection_count == 1

    sdk.close()
    sdk.close()
    assert arm.disconnect_calls == 1
    assert sdk.session_evidence["lifecycle"] == "closed"
    with pytest.raises(RuntimeError, match="session is closed"):
        sdk.execute("state", {})
    assert arm.connect_calls == 1


def test_connect_failure_is_terminal_not_an_infinite_retry(tmp_path) -> None:
    arm = FakeConnectedArm(SOARM101Config(robot_id="so101"))

    def failed_connect():
        arm.connect_calls += 1
        raise RobotConnectionError("fabricated serial failure")

    arm.connect = failed_connect
    sdk, _ = make_executor(tmp_path, arm)
    with pytest.raises(RobotConnectionError, match="fabricated serial failure"):
        sdk._arm()
    assert sdk.session_evidence["lifecycle"] == "faulted"
    assert sdk.session_evidence["fault_kind"] == "RobotConnectionError"
    with pytest.raises(RuntimeError, match="human restart required"):
        sdk._arm()
    assert arm.connect_calls == 1
    assert arm.disconnect_calls == 1
    sdk.close()


def test_calibration_change_rejects_reuse_and_revokes_physical_lease(tmp_path) -> None:
    cfg = SOARM101Config(port="/dev/fake0", robot_id="so101")
    arm = FakeConnectedArm(cfg)
    sdk, auth = make_executor(tmp_path, arm, physical=True)
    sdk._arm()
    auth.issue(robot_id="so101", calibration_id="cal-good", minutes=1)
    assert auth.status()["armed"]
    arm.calibration_id = "cal-other"
    with pytest.raises(RobotConnectionError, match="calibration identity changed"):
        sdk._arm()
    assert sdk.session_evidence["lifecycle"] == "faulted"
    assert not auth.status()["armed"]
    with pytest.raises(RuntimeError, match="human restart required"):
        sdk.execute("capabilities", {})
    sdk.close()


def test_disconnected_session_is_terminal_and_does_not_reopen(tmp_path) -> None:
    arm = FakeConnectedArm(SOARM101Config(robot_id="so101"))
    sdk, _ = make_executor(tmp_path, arm)
    sdk._arm()
    arm.is_connected = False
    with pytest.raises(RobotConnectionError, match="device disconnected"):
        sdk._arm()
    assert sdk.session_evidence["lifecycle"] == "faulted"
    with pytest.raises(RuntimeError, match="human restart required"):
        sdk.execute("state", {})
    assert arm.connect_calls == 1
    sdk.close()


def test_transport_fault_revokes_authority_but_allows_existing_stop(tmp_path, monkeypatch) -> None:
    cfg = SOARM101Config(port="/dev/fake0", robot_id="so101")
    arm = FakeConnectedArm(cfg)
    sdk, auth = make_executor(tmp_path, arm, physical=True)
    sdk._arm()
    auth.issue(robot_id="so101", calibration_id="cal-good", minutes=1)

    def bad_read(action, request):
        raise CommunicationError("fabricated packet loss")

    monkeypatch.setattr(sdk, "_execute_action", bad_read)
    with pytest.raises(CommunicationError, match="fabricated packet loss"):
        sdk.execute("state", {})
    assert sdk.session_evidence["lifecycle"] == "faulted"
    assert not auth.status()["armed"]
    assert arm.connect_calls == 1
    with pytest.raises(RuntimeError, match="human restart required"):
        sdk.execute("capabilities", {})

    # STOP still reaches an already connected session, even after the
    # authority lease has been revoked. It may never connect a new device.
    monkeypatch.undo()
    result = sdk.execute("stop", {})
    assert result["action"] == "stop"
    assert arm.stop_calls == 1
    assert arm.connect_calls == 1
    sdk.close()


def test_shutdown_rejects_queued_requests_while_first_action_finishes(
    tmp_path, monkeypatch,
) -> None:
    arm = FakeConnectedArm(SOARM101Config(robot_id="so101"))
    sdk, _ = make_executor(tmp_path, arm)
    sdk._arm()
    started = threading.Event()
    release = threading.Event()
    completed = []
    errors = []

    def slow_state(action, request):
        started.set()
        if not release.wait(timeout=3):
            raise AssertionError("test movement did not release")
        return {"state": "done"}

    monkeypatch.setattr(sdk, "_execute_action", slow_state)

    def worker():
        try:
            completed.append(sdk.execute("state", {}))
        except Exception as exc:
            errors.append(exc)

    motion = threading.Thread(target=worker)
    motion.start()
    assert started.wait(timeout=2)
    closed = threading.Thread(target=sdk.close)
    closed.start()
    try:
        for _ in range(200):
            if sdk.session_evidence["lifecycle"] == "closing":
                break
            time.sleep(0.005)
        assert sdk.session_evidence["lifecycle"] == "closing"
        # The direct owner gate fails immediately without queuing behind the
        # long-running action's dispatch lock.
        with pytest.raises(RuntimeError, match="session is closed"):
            sdk._arm()
    finally:
        release.set()
        motion.join(timeout=3)
        closed.join(timeout=3)

    assert not errors and not motion.is_alive() and not closed.is_alive()
    assert len(completed) == 1
    assert sdk.session_evidence["lifecycle"] == "closed"
    assert arm.disconnect_calls == 1


def test_stop_can_reach_owned_arm_while_close_waits_for_active_action(
    tmp_path, monkeypatch,
) -> None:
    arm = FakeConnectedArm(SOARM101Config(robot_id="so101"))
    sdk, _ = make_executor(tmp_path, arm)
    sdk._arm()
    entered = threading.Event()
    released = threading.Event()
    motion_error = []

    def simulated_action(action, request):
        entered.set()
        if not released.wait(timeout=3):
            raise RuntimeError("simulated move did not release")
        return {"state": "done"}

    original = sdk._execute_action
    monkeypatch.setattr(sdk, "_execute_action", simulated_action)

    def moving():
        try:
            sdk.execute("state", {})
        except Exception as exc:
            motion_error.append(exc)

    t = threading.Thread(target=moving)
    t.start()
    assert entered.wait(timeout=2)
    closing = threading.Thread(target=sdk.close)
    closing.start()
    try:
        for _ in range(200):
            if sdk.session_evidence["lifecycle"] == "closing":
                break
            time.sleep(0.005)
        assert sdk.session_evidence["lifecycle"] == "closing"
        # Restore canonical STOP dispatch, which does not acquire the long
        # ordinary-action lock. This must work even after closing begins.
        monkeypatch.setattr(sdk, "_execute_action", original)
        assert sdk.execute("stop", {})["completed"] is True
        assert arm.stop_calls == 1
    finally:
        released.set()
        t.join(timeout=3)
        closing.join(timeout=3)
    assert not motion_error
    assert not closing.is_alive() and not t.is_alive()
    assert sdk.session_evidence["lifecycle"] == "closed"


def test_factory_failure_latches_without_retry(tmp_path) -> None:
    config = SOARM101Config(robot_id="so101")
    attempts = []

    def unavailable_factory():
        attempts.append(1)
        raise RobotConnectionError("factory cannot acquire device")

    sdk = SDKAgentExecutor(
        config=config, rates=AgentMotionRates(), simulation=True,
        authority_store=AgentAuthorityStore(tmp_path / "authority.json"),
        arm_factory=unavailable_factory,
    )
    with pytest.raises(RobotConnectionError, match="cannot acquire device"):
        sdk._arm()
    assert sdk.session_evidence["lifecycle"] == "faulted"
    with pytest.raises(RuntimeError, match="human restart required"):
        sdk._arm()
    assert len(attempts) == 1
    sdk.close()
