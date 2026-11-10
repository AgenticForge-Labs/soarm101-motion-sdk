from __future__ import annotations

import time
from types import SimpleNamespace

import pytest

from soarm101_motion import SOARM101
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.gui.manual_follower import JointTargetFollower
from soarm101_motion.gui.worker import RobotWorker
from soarm101_motion.kinematics import SO101KinematicModel


class _Timer:
    def __init__(self) -> None:
        self.stopped = False

    def stop(self) -> None:
        self.stopped = True

    def setInterval(self, _interval: int) -> None:
        pass

    def start(self) -> None:
        pass


class _Arm:
    is_connected = True

    def __init__(self, *, fail_stream: bool = False, fail_hold: bool = False) -> None:
        self.fail_stream = fail_stream
        self.fail_hold = fail_hold
        self.holds = 0
        self.samples: list[dict[str, float]] = []
        self.model = SO101KinematicModel()
        self.active_tcp = self.model.tcp

    def stream_joint_target(self, target: object) -> object:
        self.samples.append(dict(target))  # type: ignore[arg-type]
        if self.fail_stream:
            raise RuntimeError("sample fault")
        return SimpleNamespace(accepted=True, message=None, final_positions=dict(target))

    def get_joint_positions(self) -> object:
        return SimpleNamespace(positions=self.samples[-1] if self.samples else {
            name: 0.0 for name in ARM_JOINTS
        })

    def get_joint_limits(self) -> dict[str, tuple[float, float]]:
        return {name: (-1.0, 1.0) for name in ARM_JOINTS}

    def stop_joint_stream(self, *, hold: bool) -> None:
        assert hold
        self.holds += 1
        if self.fail_hold:
            raise RuntimeError("hold fault")


def _active_worker(arm: _Arm) -> RobotWorker:
    worker = RobotWorker()
    worker.arm = arm  # type: ignore[assignment]
    worker._manual_timer = _Timer()  # type: ignore[assignment]
    measured = {name: 0.0 for name in ARM_JOINTS}
    worker._manual_follower = JointTargetFollower(
        measured, frequency_hz=20.0, max_speed=0.5,
        max_acceleration=1.0, max_step=0.05,
    )
    worker._manual_target = dict(measured)
    worker._manual_last_heartbeat = time.monotonic()
    return worker


def test_manual_disable_and_stale_heartbeat_both_hold() -> None:
    arm = _Arm()
    worker = _active_worker(arm)
    worker.stop_live_manual()
    assert arm.holds == 1
    assert worker._manual_follower is None

    arm = _Arm()
    worker = _active_worker(arm)
    errors: list[str] = []
    worker.error_message.connect(errors.append)
    worker._manual_last_heartbeat = time.monotonic() - 3.0
    worker._manual_follower_tick()
    assert arm.holds == 1
    assert worker._manual_follower is None
    assert any("heartbeat timed out" in message for message in errors)


def test_target_publication_replaces_values_without_queuing() -> None:
    arm = _Arm()
    worker = _active_worker(arm)
    for angle in (2.0, 12.0, -4.0):
        worker.publish_live_manual_target(
            {name: angle if name == "shoulder_pan" else 0.0 for name in ARM_JOINTS}
        )
    worker._manual_follower_tick()
    assert len(arm.samples) == 1
    assert arm.samples[0]["shoulder_pan"] < 0.0


def test_partial_targets_merge_over_exact_measured_start() -> None:
    arm = _Arm()
    worker = RobotWorker()
    worker.arm = arm  # type: ignore[assignment]
    worker._manual_session_id = 12
    measured = {name: 0.12345 for name in ARM_JOINTS}
    worker._manual_follower = JointTargetFollower(
        measured, frequency_hz=20.0, max_speed=0.5, max_acceleration=1.0, max_step=0.05,
    )
    worker._manual_target = dict(measured)
    worker._manual_last_heartbeat = time.monotonic()
    worker.publish_live_manual_target({"session_id": 12, "gripper": 0.37})
    worker._manual_follower_tick()
    assert arm.samples[-1] == measured
    assert worker._manual_target == measured


def test_stale_target_and_stop_do_not_affect_new_session() -> None:
    arm = _Arm()
    worker = _active_worker(arm)
    worker._manual_session_id = 9
    follower = worker._manual_follower
    worker.publish_live_manual_target({"session_id": 8, "shoulder_pan": 25.0})
    worker.stop_live_manual(8)
    assert worker._manual_follower is follower
    assert worker._manual_pending_degrees is None


def test_manual_admission_failure_emits_one_session_end() -> None:
    arm = SOARM101.simulated()
    arm.connect()
    worker = RobotWorker()
    worker.arm = arm
    ended: list[dict[str, object]] = []
    worker.live_manual_ended.connect(ended.append)
    worker.start_live_manual({
        "session_id": 41, "speed_deg_s": 10.0, "acceleration_deg_s2": 100.0,
    })
    assert len(ended) == 1
    assert ended[0]["session_id"] == 41
    worker.stop_live_manual(41)
    assert len(ended) == 1
    arm.disconnect()


def test_manual_stream_uses_controller_captured_baseline_without_unrequested_motion(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A changed second measurement must not make startup target the earlier read."""
    arm = SOARM101.simulated()
    arm.connect()
    arm.enable()
    backend = arm.backend
    read_positions = backend.read_joint_positions
    reads = 0
    controller_baseline: dict[str, float] = {}
    earlier_read_calls = 0

    def differing_reads() -> dict[str, float]:
        nonlocal reads, controller_baseline
        reads += 1
        positions = dict(read_positions())
        controller_baseline = positions.copy()
        return positions

    def differing_earlier_read() -> object:
        nonlocal earlier_read_calls
        earlier_read_calls += 1
        positions = dict(read_positions())
        positions["shoulder_pan"] += 0.001
        return SimpleNamespace(positions=positions)

    monkeypatch.setattr(backend, "read_joint_positions", differing_reads)
    monkeypatch.setattr(arm, "get_joint_positions", differing_earlier_read)
    worker = RobotWorker()
    worker.arm = arm
    worker._manual_timer = _Timer()  # type: ignore[assignment]
    ready: list[dict[str, object]] = []
    worker.live_manual_ready.connect(ready.append)
    worker.start_live_manual({
        "session_id": 42, "speed_deg_s": 10.0, "acceleration_deg_s2": 100.0,
    })

    assert reads == 1  # stream startup's single guarded measurement
    assert earlier_read_calls == 0
    earlier_worker_read = {**controller_baseline, "shoulder_pan":
                           controller_baseline["shoulder_pan"] + 0.001}
    assert earlier_worker_read != controller_baseline
    assert worker._manual_target == controller_baseline
    assert worker._manual_follower is not None
    assert worker._manual_follower.position == controller_baseline
    assert ready[0]["joints_deg"]["shoulder_pan"] == pytest.approx(
        controller_baseline["shoulder_pan"] * 180.0 / 3.141592653589793
    )
    worker._manual_follower_tick()
    assert arm.motion._joint_stream is not None
    assert arm.motion._joint_stream.last_command == controller_baseline
    assert worker._manual_gripper_desired == pytest.approx(float(ready[0]["gripper"]))
    assert not worker._manual_gripper_enabled
    worker.stop_live_manual(42)
    arm.disconnect()


def test_manual_stream_publishes_measured_joint_tcp_feedback() -> None:
    arm = _Arm()
    worker = _active_worker(arm)
    joint_events: list[dict[str, float]] = []
    live_events: list[dict[str, object]] = []
    worker.joint_measurements.connect(joint_events.append)
    worker.live_measurements.connect(live_events.append)
    target = {name: 0.0 for name in ARM_JOINTS}
    target["wrist_flex"] = 15.0
    worker.publish_live_manual_target(target)
    worker._manual_follower_tick()
    assert joint_events
    assert live_events[-1]["joints_deg"] == joint_events[-1]
    assert "pose_mm_deg" in live_events[-1]
    assert "tcp_xyz_rpy" in live_events[-1]
    assert joint_events[-1]["wrist_flex"] > 0.0


def test_manual_stream_fault_fails_closed_and_reports_hold_failure() -> None:
    arm = _Arm(fail_stream=True, fail_hold=True)
    worker = _active_worker(arm)
    errors: list[str] = []
    worker.error_message.connect(errors.append)
    worker._manual_follower_tick()
    assert arm.holds == 1
    assert worker._manual_follower is None
    assert any("sample fault" in message for message in errors)
    assert any("hold fault" in message for message in errors)
    assert worker._manual_hold_unconfirmed
    with pytest.raises(RuntimeError, match="HOLD is unconfirmed"):
        worker._require_motion_available()
    worker.move_joints({
        "joints_deg": {name: 0.0 for name in ARM_JOINTS},
        "speed_deg_s": 10.0,
        "acceleration_deg_s2": 100.0,
    })
    worker.enable()
    assert any("motion is latched off" in message for message in errors)
    assert any("before enabling torque" in message for message in errors)
    busy: list[bool] = []
    worker.busy_changed.connect(busy.append)
    worker._process_handles()
    assert busy[-1] is True


def test_failed_hold_latch_survives_poll_and_only_explicit_verified_stop_recovers() -> None:
    class RecoverableArm(_Arm):
        def __init__(self) -> None:
            super().__init__(fail_hold=True)
            self.stop_fails = True
            self.stop_calls = 0

        def stop(self) -> None:
            self.stop_calls += 1
            if self.stop_fails:
                raise RuntimeError("arm STOP fault")

        def get_state(self) -> object:
            return type("State", (), {"connected": True, "torque_enabled": True})()

    arm = RecoverableArm()
    worker = _active_worker(arm)
    worker.stop_live_manual()
    assert worker._manual_hold_unconfirmed
    assert worker._manual_follower is None
    with pytest.raises(RuntimeError, match="HOLD is unconfirmed"):
        worker._require_motion_available()

    arm.stop_fails = False
    worker.stop()
    assert arm.stop_calls == 1
    assert not worker._manual_hold_unconfirmed


def test_failed_stop_retry_keeps_manual_fault_latched() -> None:
    class FailedArm(_Arm):
        def stop(self) -> None:
            raise RuntimeError("arm STOP fault")

        def get_state(self) -> object:
            return type("State", (), {"connected": True, "torque_enabled": True})()

    arm = FailedArm(fail_hold=True)
    worker = _active_worker(arm)
    worker.stop_live_manual()
    worker.stop()
    assert worker._manual_hold_unconfirmed
    with pytest.raises(RuntimeError, match="HOLD is unconfirmed"):
        worker._require_motion_available()


def test_relax_aborts_when_live_hold_is_unconfirmed() -> None:
    class RelaxArm(_Arm):
        relaxed = False

        def relax(self) -> None:
            self.relaxed = True

    arm = RelaxArm(fail_hold=True)
    worker = _active_worker(arm)
    errors: list[str] = []
    worker.error_message.connect(errors.append)
    worker.relax()
    assert arm.holds == 1
    assert worker._manual_hold_unconfirmed
    assert not arm.relaxed
    assert any("relaxation aborted" in message for message in errors)
    worker.relax()
    assert not arm.relaxed
    assert worker._manual_hold_unconfirmed


@pytest.mark.parametrize("shutdown", [False, True], ids=["disconnect", "close-shutdown"])
def test_disconnect_and_close_preserve_unconfirmed_hold_latch(monkeypatch, shutdown: bool) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication

    app = QApplication.instance() or QApplication([])
    arm = SOARM101.simulated()
    arm.connect()
    arm.enable()
    worker = RobotWorker()
    worker.arm = arm
    worker.start()
    worker.start_live_manual({"speed_deg_s": 10.0, "acceleration_deg_s2": 100.0})

    def fail_hold(*, hold: bool) -> None:
        assert hold
        raise RuntimeError("simulated unconfirmed HOLD")

    arm.stop_joint_stream = fail_hold  # type: ignore[method-assign]
    (worker.shutdown if shutdown else worker.disconnect_robot)()
    assert worker._manual_hold_unconfirmed
    assert worker.arm is None
    with pytest.raises(RuntimeError, match="HOLD is unconfirmed"):
        worker._require_motion_available()
    if worker._timer is not None:
        worker._timer.stop()
    app.processEvents()


def test_simulated_worker_disconnect_stops_the_manual_stream(monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication

    app = QApplication.instance() or QApplication([])
    arm = SOARM101.simulated()
    arm.connect()
    arm.enable()
    worker = RobotWorker()
    worker.arm = arm
    worker.start()
    worker.start_live_manual({"speed_deg_s": 10.0, "acceleration_deg_s2": 100.0})
    assert arm.motion.is_streaming
    worker.stop()
    assert not arm.motion.is_streaming
    worker.start_live_manual({"speed_deg_s": 10.0, "acceleration_deg_s2": 100.0})
    assert arm.motion.is_streaming
    worker.disconnect_robot()
    assert not arm.is_connected
    assert worker._manual_follower is None
    if worker._timer is not None:
        worker._timer.stop()
    app.processEvents()
