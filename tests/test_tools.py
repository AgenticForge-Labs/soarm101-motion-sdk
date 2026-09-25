import numpy as np
import pytest

from soarm101_motion import CameraTool, Pose, SOARM101, ToolAssembly
from soarm101_motion.tools import SO101Gripper
from soarm101_motion.exceptions import InvalidCommandError, MotionTimeoutError
from soarm101_motion.hardware import SimulationBackend


def test_camera_tool_tcp() -> None:
    camera_pose = Pose(np.array([0.01, 0.02, -0.03]), np.eye(3))
    with SOARM101.simulated(tool=CameraTool(camera_pose)) as arm:
        arm.enable()
        assert np.allclose(arm.active_tcp.position, camera_pose.position)
        arm.set_active_tcp("camera")


def test_tool_assembly_binds_primary_gripper() -> None:
    gripper = SO101Gripper()
    camera = CameraTool(Pose(np.array([0.01, 0.0, -0.04]), np.eye(3)))
    assembly = ToolAssembly(primary=gripper, attachments={"camera": camera})
    with SOARM101.simulated(tool=assembly) as arm:
        arm.enable()
        gripper.close()
        assert gripper.is_closed
        arm.set_active_tcp("camera")


def test_gripper_begin_opening_sends_goal_without_starting_polling() -> None:
    backend = SimulationBackend()
    backend.connect()
    backend.enable_torque()
    backend.write_tool_position("so101_gripper", 0.2)
    gripper = SO101Gripper(backend=backend)

    gripper.begin_opening(0.8, speed_raw=100)

    assert gripper.get_position() == pytest.approx(0.8)
    assert not gripper.is_moving
    with pytest.raises(InvalidCommandError, match="cannot command.*close"):
        gripper.begin_opening(0.1)
    backend.disconnect()


def test_gripper_long_travel_can_finish_after_old_three_second_deadline() -> None:
    class SlowBackend(SimulationBackend):
        realtime = True

        def __init__(self) -> None:
            super().__init__(realtime=True)
            self._tool_positions["so101_gripper"] = 0.0
            self.target = 0.0
            self.reads = 0

        def write_tool_position(self, actuator: str, position: float, **kwargs: object) -> None:
            self.target = position

        def read_tool_position(self, actuator: str) -> float:
            self.reads += 1
            self._tool_positions[actuator] = min(self.target, self._tool_positions[actuator] + 0.004)
            return self._tool_positions[actuator]

    backend = SlowBackend()
    backend.connect()
    backend.enable_torque()
    gripper = SO101Gripper(backend=backend)
    assert gripper.default_timeout_s > 3.0
    result = gripper.open()
    assert result.completed
    assert backend.reads > 200


def test_gripper_stall_stops_motor() -> None:
    class StalledBackend(SimulationBackend):
        realtime = True

        def __init__(self) -> None:
            super().__init__(realtime=True)
            self._tool_positions["so101_gripper"] = 0.0

        def write_tool_position(self, actuator: str, position: float, **kwargs: object) -> None:
            pass

    backend = StalledBackend()
    backend.connect()
    backend.enable_torque()
    with pytest.raises(MotionTimeoutError):
        SO101Gripper(backend=backend).open(timeout=0.05)
    assert backend.stop_count >= 1
