"""Default GUI diagnostics remain useful without touching physical motors."""

from __future__ import annotations

import json
import time
from types import SimpleNamespace

import pytest

from soarm101_motion.constants import ARM_JOINTS


def test_session_log_flushes_queued_events(tmp_path) -> None:
    from soarm101_motion.gui import session_log

    path = session_log.configure(directory=tmp_path)
    assert path is not None
    assert session_log.current_path() == path
    session_log.record("diagnostic_sample", sample=1)
    session_log.close()
    try:
        events = [json.loads(line) for line in path.read_text().splitlines()]
        assert [event["event"] for event in events] == ["session_start", "diagnostic_sample"]
    finally:
        session_log.configure(enabled=False)


def test_calibration_color_advances_from_red_to_green() -> None:
    pytest.importorskip("PySide6")
    from soarm101_motion.gui.calibration_progress import sweep_color

    assert sweep_color(0.0).hue() < sweep_color(0.5).hue() < sweep_color(1.0).hue()
    assert sweep_color(-1.0) == sweep_color(0.0)
    assert sweep_color(2.0) == sweep_color(1.0)


def test_teleop_records_per_sample_target_and_measured_pose(monkeypatch) -> None:
    pytest.importorskip("PySide6")
    from soarm101_motion.config import SOARM101Config
    from soarm101_motion.gui.worker import RobotWorker

    events = []
    monkeypatch.setattr(
        "soarm101_motion.gui.worker.record_session",
        lambda name, **fields: events.append((name, fields)),
    )
    joints = {name: 0.0 for name in ARM_JOINTS}

    class FakeArm:
        is_connected = True
        config = SOARM101Config()
        motion = SimpleNamespace(is_streaming=False)
        tool = SimpleNamespace(get_position=lambda: 0.5)

        def get_joint_positions(self):
            return SimpleNamespace(positions=joints)

        def get_joint_limits(self):
            return {name: (-2.0, 2.0) for name in ARM_JOINTS}

        def start_joint_stream(self, *, frequency_hz):
            assert frequency_hz == 10.0

        def stream_joint_target(self, command, *, gripper, gripper_speed_raw=None):
            assert gripper_speed_raw == 500
            return SimpleNamespace(final_positions=dict(command), message="accepted")

    worker = RobotWorker()
    worker.arm = FakeArm()
    worker.start_teleop({
        "mode": "relative",
        "frequency_hz": 10.0,
        "leader_joints_rad": joints,
        "mirror_gripper": True,
    })
    assert worker._teleop is not None
    sample = {"timestamp": time.perf_counter(), "joints_rad": joints, "gripper": 0.5}
    worker.apply_teleop_sample(sample)
    frames = [fields for name, fields in events if name == "teleop_frame"]
    assert len(frames) == 1
    assert frames[0]["leader_joints_rad"] == joints
    assert frames[0]["command_joints_rad"] == joints
    assert frames[0]["actual_joints_rad"] == joints
    assert frames[0]["following_error_rad"] == joints
    assert frames[0]["processing_ms"] >= 0

    worker.set_detailed_logging(False)
    worker.apply_teleop_sample({**sample, "timestamp": time.perf_counter()})
    assert len([name for name, _ in events if name == "teleop_frame"]) == 1


def test_manual_gripper_speed_reaches_tool_move() -> None:
    pytest.importorskip("PySide6")
    from soarm101_motion.config import SOARM101Config
    from soarm101_motion.gui.worker import RobotWorker

    calls = []
    worker = RobotWorker()
    worker.arm = SimpleNamespace(
        config=SOARM101Config(),
        tool=SimpleNamespace(move=lambda position, **kwargs: calls.append((position, kwargs))),
    )
    worker._require_motion_available = lambda: worker.arm
    worker._track = lambda *_args: None
    worker.move_gripper({"position": 0.3, "gripper_speed_multiplier": 5.0})
    assert calls == [(0.3, {"speed_raw": 1250, "wait": False})]


def test_gripper_speed_presets() -> None:
    from soarm101_motion.gui.teleop_rate import gripper_speed_raw

    assert [gripper_speed_raw(250, multiplier) for multiplier in (1.0, 2.0, 5.0)] == [
        250, 500, 1250
    ]
    with pytest.raises(ValueError, match="unknown"):
        gripper_speed_raw(250, 10.0)


def test_teleop_torque_enable_retries_missing_reply_after_torque_off(monkeypatch) -> None:
    pytest.importorskip("PySide6")
    from soarm101_motion.exceptions import CommunicationError
    from soarm101_motion.gui.worker import RobotWorker

    events = []
    monkeypatch.setattr(
        "soarm101_motion.gui.worker.record_session",
        lambda name, **fields: events.append((name, fields)),
    )

    class FakeArm:
        backend = SimpleNamespace(disable_torque=lambda: events.append(("torque_off", {})))

        def enable(self):
            events.append(("enable", {}))
            if sum(name == "enable" for name, _ in events) < 3:
                raise CommunicationError(
                    "write Torque_Enable on shoulder_lift: There is no status packet!"
                )

    worker = RobotWorker()
    worker._enable_follower_for_teleop(FakeArm())
    assert [name for name, _ in events if name in {"enable", "torque_off"}] == [
        "enable", "torque_off", "enable", "torque_off", "enable"
    ]
    assert len([name for name, _ in events if name == "teleop_torque_enable_retry"]) == 2


def test_teleop_torque_enable_does_not_retry_other_faults() -> None:
    pytest.importorskip("PySide6")
    from soarm101_motion.exceptions import CommunicationError
    from soarm101_motion.gui.worker import RobotWorker

    calls = []

    class FakeArm:
        backend = SimpleNamespace(disable_torque=lambda: calls.append("torque_off"))

        def enable(self):
            calls.append("enable")
            raise CommunicationError("write Torque_Enable on shoulder_lift: voltage fault")

    with pytest.raises(CommunicationError, match="voltage fault"):
        RobotWorker()._enable_follower_for_teleop(FakeArm())
    assert calls == ["enable"]


def test_teleop_torque_enable_fails_after_two_retries() -> None:
    pytest.importorskip("PySide6")
    from soarm101_motion.exceptions import CommunicationError
    from soarm101_motion.gui.worker import RobotWorker

    calls = []

    class FakeArm:
        backend = SimpleNamespace(disable_torque=lambda: calls.append("torque_off"))

        def enable(self):
            calls.append("enable")
            raise CommunicationError("write Torque_Enable: There is no status packet!")

    with pytest.raises(CommunicationError, match="no status packet"):
        RobotWorker()._enable_follower_for_teleop(FakeArm())
    assert calls == ["enable", "torque_off"] * 3


def test_teleop_gripper_contact_latch_does_not_stop_arm_stream(monkeypatch) -> None:
    pytest.importorskip("PySide6")
    from soarm101_motion.config import SOARM101Config
    from soarm101_motion.gui.worker import RobotWorker

    events = []
    monkeypatch.setattr(
        "soarm101_motion.gui.worker.record_session",
        lambda name, **fields: events.append((name, fields)),
    )
    joints = {name: 0.0 for name in ARM_JOINTS}

    class FakeArm:
        is_connected = True
        config = SOARM101Config()
        motion = SimpleNamespace(is_streaming=False)

        def __init__(self):
            self.gripper_targets = []
            self.tool = SimpleNamespace(get_position=lambda: 0.5)

        def get_joint_positions(self):
            return SimpleNamespace(positions=joints)

        def get_joint_limits(self):
            return {name: (-2.0, 2.0) for name in ARM_JOINTS}

        def start_joint_stream(self, *, frequency_hz):
            assert frequency_hz == 10.0

        def stream_joint_target(self, command, *, gripper, gripper_speed_raw=None):
            self.gripper_targets.append(gripper)
            return SimpleNamespace(final_positions=dict(command), message="accepted")

    worker = RobotWorker()
    worker.arm = FakeArm()
    worker.start_teleop({
        "mode": "relative",
        "frequency_hz": 10.0,
        "leader_joints_rad": joints,
        "mirror_gripper": True,
    })
    for gripper in (0.5, *([0.0] * 6), 0.1, 0.8):
        worker.apply_teleop_sample({
            "timestamp": time.perf_counter(),
            "joints_rad": joints,
            "gripper": gripper,
        })

    assert worker._teleop is not None
    assert worker.arm.gripper_targets[6] == pytest.approx(0.505)
    assert worker.arm.gripper_targets[-1] > 0.5
    assert any(name == "teleop_gripper_contact_latched" for name, _ in events)
    assert any(name == "teleop_gripper_contact_released" for name, _ in events)


@pytest.mark.parametrize(
    ("follower_start", "leader_gripper", "stages_tool", "opening_arrives"),
    [(0.2, 0.8, True, True), (0.2, 0.8, True, False), (0.8, 0.2, False, False)],
)
def test_teleop_stages_opening_and_guards_closing(
    monkeypatch, follower_start, leader_gripper, stages_tool, opening_arrives
) -> None:
    pytest.importorskip("PySide6")
    from soarm101_motion.config import SOARM101Config
    from soarm101_motion.gui.worker import RobotWorker
    from soarm101_motion.motion import MotionHandle

    monkeypatch.setattr("soarm101_motion.gui.worker.record_session", lambda *args, **kwargs: None)
    joints = {name: 0.0 for name in ARM_JOINTS}
    events = []

    class FakeTool:
        position = follower_start

        def get_position(self):
            return self.position

        def begin_opening(self, target, *, speed_raw):
            events.append(("begin_opening", target, speed_raw))
            if opening_arrives:
                self.position = target

        def move(self, target, *, wait, timeout):
            assert not wait
            handle = MotionHandle(lambda _cancel: setattr(self, "position", target) or target)
            handle.start()
            return handle

    class FakeArm:
        is_connected = True
        config = SOARM101Config()
        backend = SimpleNamespace(calibration=None)
        motion = SimpleNamespace(is_streaming=False)

        def __init__(self):
            self.tool = FakeTool()
            self.gripper_targets = []

        def get_state(self):
            return SimpleNamespace(torque_enabled=True)

        def get_joint_positions(self):
            return SimpleNamespace(positions=joints)

        def get_joint_limits(self):
            return {name: (-2.0, 2.0) for name in ARM_JOINTS}

        def move_joints(self, target, **kwargs):
            assert kwargs["servo_speed_raw"] == 0
            assert kwargs["servo_acceleration_raw"] == 254
            events.append(("move_joints", target))
            handle = MotionHandle(lambda _cancel: target)
            handle.start()
            return handle

        def start_joint_stream(self, *, frequency_hz):
            self.motion.is_streaming = True

        def stream_joint_target(self, command, *, gripper, gripper_speed_raw=None):
            self.gripper_targets.append(gripper)
            return SimpleNamespace(final_positions=dict(command), message="accepted")

    worker = RobotWorker()
    worker.arm = FakeArm()
    worker.start_teleop({
        "mode": "relative",
        "frequency_hz": 10.0,
        "leader_joints_rad": joints,
        "leader_gripper": leader_gripper,
        "mirror_gripper": True,
        "align_follower": True,
    })
    assert events[0][0] == "move_joints"
    assert (len(events) == 2 and events[1][0] == "begin_opening") == stages_tool
    worker._handles[0][1].wait(1)
    worker._process_handles()
    if stages_tool and not opening_arrives:
        assert worker._handles[0][0] == "teleop gripper alignment"
        worker._handles[0][1].wait(1)
        worker._process_handles()
    if stages_tool:
        assert worker.arm.tool.position == pytest.approx(leader_gripper)
    else:
        assert worker.arm.tool.position == pytest.approx(follower_start)
    assert worker._teleop is not None

    worker.apply_teleop_sample({
        "timestamp": time.perf_counter(),
        "joints_rad": joints,
        "gripper": leader_gripper - 0.1,
    })
    assert worker.arm.gripper_targets[-1] == pytest.approx(0.7 if stages_tool else 0.68)


def test_gripper_mirrors_absolute_leader_value_when_joint_mapping_is_relative(monkeypatch) -> None:
    pytest.importorskip("PySide6")
    from soarm101_motion.config import SOARM101Config
    from soarm101_motion.gui.worker import RobotWorker

    monkeypatch.setattr("soarm101_motion.gui.worker.record_session", lambda *args, **kwargs: None)
    joints = {name: 0.0 for name in ARM_JOINTS}

    class FakeArm:
        is_connected = True
        config = SOARM101Config()
        motion = SimpleNamespace(is_streaming=False)

        def __init__(self):
            self.tool = SimpleNamespace(get_position=lambda: 0.8)
            self.gripper_targets = []

        def get_joint_positions(self):
            return SimpleNamespace(positions=joints)

        def get_joint_limits(self):
            return {name: (-2.0, 2.0) for name in ARM_JOINTS}

        def start_joint_stream(self, *, frequency_hz):
            pass

        def stream_joint_target(self, command, *, gripper, gripper_speed_raw=None):
            self.gripper_targets.append(gripper)
            return SimpleNamespace(final_positions=dict(command), message="accepted")

    worker = RobotWorker()
    worker.arm = FakeArm()
    worker.start_teleop({
        "mode": "relative", "frequency_hz": 10.0,
        "leader_joints_rad": joints, "mirror_gripper": True,
    })
    worker.apply_teleop_sample({
        "timestamp": time.perf_counter(), "joints_rad": joints, "gripper": 0.2,
    })
    assert worker.arm.gripper_targets[-1] == pytest.approx(0.68)


def test_repeated_poll_fault_is_reported_once_until_readout_recovers() -> None:
    pytest.importorskip("PySide6")
    from soarm101_motion.gui.worker import RobotWorker

    class FakeArm:
        is_connected = True
        motion = SimpleNamespace(is_streaming=False)

        def get_state(self):
            raise RuntimeError("so101_gripper overload")

    worker = RobotWorker()
    worker.arm = FakeArm()
    errors = []
    worker.error_message.connect(errors.append)
    worker.poll()
    worker.poll()
    assert errors == ["read state: so101_gripper overload"]


def test_setup_shows_default_detailed_log_option(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication
    from soarm101_motion.gui import session_log
    from soarm101_motion.gui.window import MainWindow

    app = QApplication.instance() or QApplication([])
    path = session_log.configure(directory=tmp_path)
    window = MainWindow(simulation=True)
    try:
        assert window.detailed_logging_check.isChecked()
        assert window.detailed_logging_check.isEnabled()
        assert window.session_log_path.text() == str(path)
    finally:
        window.close()
        app.processEvents()
        session_log.configure(enabled=False)
