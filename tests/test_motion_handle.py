import time

import pytest

from soarm101_motion import SOARM101
from soarm101_motion.exceptions import MotionCancelledError


def test_nonblocking_handle_completes() -> None:
    arm = SOARM101.simulated(realtime=False)
    with arm:
        arm.enable()
        handle = arm.move_joints([0.1, -0.2, 0.2, 0, 0], wait=False)
        result = handle.wait(timeout=2)
        assert result.completed
        assert handle.done


def test_cancel_realtime_motion() -> None:
    arm = SOARM101.simulated(realtime=True)
    with arm:
        arm.enable()
        handle = arm.move_joints([0.5, -0.8, 0.8, 0.5, 0.3], speed=0.1, wait=False)
        time.sleep(0.03)
        handle.cancel()
        with pytest.raises(MotionCancelledError):
            handle.wait(timeout=2)
