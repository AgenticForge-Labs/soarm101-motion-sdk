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

        def stream_joint_target(self, command, *, gripper):
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

        def stream_joint_target(self, command, *, gripper):
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
    ("follower_start", "leader_gripper", "stages_tool"),
    [(0.2, 0.8, True), (0.8, 0.2, False)],
)
def test_teleop_stages_opening_and_guards_closing(
    monkeypatch, follower_start, leader_gripper, stages_tool
) -> None:
    pytest.importorskip("PySide6")
    from soarm101_motion.config import SOARM101Config
    from soarm101_motion.gui.worker import RobotWorker
    from soarm101_motion.motion import MotionHandle

    monkeypatch.setattr("soarm101_motion.gui.worker.record_session", lambda *args, **kwargs: None)
    joints = {name: 0.0 for name in ARM_JOINTS}

    class FakeTool:
        position = follower_start

        def get_position(self):
            return self.position

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
            handle = MotionHandle(lambda _cancel: target)
            handle.start()
            return handle

        def start_joint_stream(self, *, frequency_hz):
            self.motion.is_streaming = True

        def stream_joint_target(self, command, *, gripper):
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
    worker._handles[0][1].wait(1)
    worker._process_handles()
    if stages_tool:
        assert worker.arm.tool.position == pytest.approx(leader_gripper)
        assert worker._handles[0][0] == "teleop gripper alignment"
        worker._handles[0][1].wait(1)
        worker._process_handles()
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

        def stream_joint_target(self, command, *, gripper):
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
