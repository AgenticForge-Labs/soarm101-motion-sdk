from __future__ import annotations

from math import radians

import pytest

from soarm101_motion import SOARM101, SOARM101Config
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.hardware import SimulationBackend


def _arm() -> SOARM101:
    arm = SOARM101(
        SOARM101Config(
            enable_workspace_checks=False,
            effort_safety_enabled=False,
            max_joint_speed=2.0,
            max_joint_acceleration=10.0,
        ),
        backend=SimulationBackend(realtime=False),
    )
    arm.connect()
    return arm


def test_worker_pose_sync_latches_relaxed_destination_and_matches_gripper() -> None:
    pytest.importorskip("PySide6")
    from soarm101_motion.gui.worker import RobotWorker

    arm = _arm()
    writes: list[tuple[str, float, int | None]] = []
    original_write = arm.backend.write_tool_position

    def capture_write(actuator, position, *, speed_raw=None, acceleration_raw=None):
        writes.append((actuator, float(position), speed_raw))
        original_write(
            actuator,
            position,
            speed_raw=speed_raw,
            acceleration_raw=acceleration_raw,
        )

    arm.backend.write_tool_position = capture_write  # type: ignore[method-assign]
    worker = RobotWorker()
    worker.arm = arm
    target_deg = {name: 0.0 for name in ARM_JOINTS}
    target_deg["shoulder_pan"] = 2.0
    try:
        assert not arm.get_state().torque_enabled
        worker.synchronize_pose(
            {
                "joints_deg": target_deg,
                "gripper": 0.4,
                "include_gripper": True,
                "gripper_speed_multiplier": 2.0,
                "source": "leader",
            }
        )
        assert arm.get_state().torque_enabled
        assert len(worker._handles) == 1
        worker._handles[0][1].wait(2.0)
        worker._process_handles()

        measured = arm.get_joint_positions().positions
        assert measured["shoulder_pan"] == pytest.approx(radians(2.0), abs=1e-6)
        assert arm.tool.get_position() == pytest.approx(0.4)
        assert writes[-1] == ("so101_gripper", 0.4, 500)
    finally:
        arm.disconnect()


def test_no_motion_relink_latches_follower_without_alignment_move() -> None:
    pytest.importorskip("PySide6")
    from soarm101_motion.gui.worker import RobotWorker

    arm = _arm()
    worker = RobotWorker()
    worker.arm = arm
    before = dict(arm.get_joint_positions().positions)
    try:
        worker.start_teleop(
            {
                "mode": "relative",
                "frequency_hz": 10.0,
                "leader_joints_rad": {name: 0.3 for name in ARM_JOINTS},
                "leader_gripper": 0.5,
                "mirror_gripper": False,
                "align_follower": False,
                "latch_follower_if_relaxed": True,
            }
        )
        assert arm.get_state().torque_enabled
        assert worker._teleop is not None
        assert worker._handles == []
        assert arm.get_joint_positions().positions == pytest.approx(before)
        worker.stop_teleop()
    finally:
        arm.disconnect()
