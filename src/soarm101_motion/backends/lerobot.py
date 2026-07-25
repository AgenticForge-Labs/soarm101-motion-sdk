"""LeRobot adapter boundary.

The concrete adapter is intentionally thin and incomplete until validated against physical
SO-ARM101 hardware and the pinned LeRobot release.
"""

from soarm101_motion.exceptions import MissingDependencyError, UnsupportedCapabilityError


class LeRobotSO101Backend:
    def __init__(self, *, port: str, robot_id: str = "soarm101") -> None:
        try:
            import lerobot  # noqa: F401
        except ImportError as exc:
            raise MissingDependencyError(
                "LeRobot support requires installation with the 'lerobot' extra"
            ) from exc
        self.port = port
        self.robot_id = robot_id

    @property
    def is_connected(self) -> bool:
        return False

    def connect(self) -> None:
        raise UnsupportedCapabilityError(
            "The LeRobot backend adapter is scaffolded but not yet hardware-validated"
        )
