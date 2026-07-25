import numpy as np

from soarm101_motion import CameraTool, Pose, SOARM101, ToolAssembly
from soarm101_motion.tools import SO101Gripper


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
