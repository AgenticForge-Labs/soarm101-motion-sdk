import pytest

from soarm101_motion import SOARM101
from soarm101_motion.backends import MockSOARM101Backend
from soarm101_motion.exceptions import CommunicationError, RobotConnectionError


def test_mock_vertical_slice() -> None:
    backend = MockSOARM101Backend()
    arm = SOARM101(backend=backend)
    arm.connect()
    result = arm.move_joints({"shoulder_pan": 0.25})
    assert result.completed
    assert arm.get_joint_positions().positions["shoulder_pan"] == 0.25
    arm.relax()
    arm.disconnect()
    assert not arm.is_connected


def test_move_requires_connection() -> None:
    arm = SOARM101(backend=MockSOARM101Backend())
    with pytest.raises(RobotConnectionError):
        arm.move_joints({"shoulder_pan": 0.0})


def test_backend_failure_is_translated() -> None:
    backend = MockSOARM101Backend()
    arm = SOARM101(backend=backend)
    arm.connect()
    backend.fail_next_command = True
    with pytest.raises(CommunicationError):
        arm.move_joints({"shoulder_pan": 0.0})
