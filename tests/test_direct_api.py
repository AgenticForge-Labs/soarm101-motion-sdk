import pytest

from soarm101_motion import SOArmAPI


def test_direct_api_motion_enable_and_gripper() -> None:
    arm = SOArmAPI.simulated()
    with arm:
        arm.motion_enable(True)
        assert arm.get_state().torque_enabled
        arm.set_gripper_position(0.25)
        assert arm.get_gripper_position() == pytest.approx(0.25, abs=0.03)
        arm.motion_enable(False)
        assert not arm.get_state().torque_enabled


def test_direct_api_pose_values_use_mm_and_degrees() -> None:
    arm = SOArmAPI.simulated()
    with arm:
        values = arm.get_position_values()
        assert len(values) == 6
        # The default model pose is reported in millimetres, unlike the native
        # Pose object which intentionally uses SI metres/radians.
        assert max(abs(value) for value in values[:3]) > 10.0


def test_direct_api_tool_frame_translation() -> None:
    arm = SOArmAPI.simulated()
    with arm:
        arm.motion_enable(True)
        before = arm.get_position()
        arm.set_tool_position(x=2.0, orientation_mode="position_only")
        after = arm.get_position()
        assert after.position.tolist() != pytest.approx(before.position.tolist(), abs=1e-6)
