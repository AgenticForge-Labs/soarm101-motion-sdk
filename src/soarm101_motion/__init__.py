"""High-level motion control for the SO-ARM101."""

from soarm101_motion.api import SOArmAPI
from soarm101_motion.arm import SOARM101
from soarm101_motion.calibration import MotorCalibration, SO101Calibration
from soarm101_motion.config import SOARM101Config
from soarm101_motion.hardware import FeetechMotorSetup, MotorSetupResult
from soarm101_motion.integration import (
    DEFAULT_BASE_FRAME,
    DEFAULT_RESOURCE_ID,
    DIRECTOR_CONTRACT_FAMILY,
    DIRECTOR_CONTRACT_VERSION,
    SOFTWARE_STOP_IS_CERTIFIED_EMERGENCY_STOP,
    IntegrationMetadata,
    integration_metadata,
)
from soarm101_motion.kinematics import IKOptions, IKSolver, SO101KinematicModel
from soarm101_motion.poses import HOME_POSE_NAME, REST_POSE_NAME, PoseLibrary, SavedPose
from soarm101_motion.primitives import MotionPrimitive, MotionPrimitiveLibrary
from soarm101_motion.sequences import MotionSequence, SequenceLibrary, SequenceRunner, SequenceStep
from soarm101_motion.tools import CameraTool, SO101Gripper, ToolAssembly
from soarm101_motion.trajectories import Trajectory, TrajectoryEntry, TrajectoryLibrary
from soarm101_motion.types import HardwareState, IKResult, JointState, MotionResult, Pose

__version__ = "0.1.0.dev8"

__all__ = [
    "CameraTool",
    "DEFAULT_BASE_FRAME",
    "DEFAULT_RESOURCE_ID",
    "DIRECTOR_CONTRACT_FAMILY",
    "DIRECTOR_CONTRACT_VERSION",
    "FeetechMotorSetup",
    "HOME_POSE_NAME",
    "HardwareState",
    "IKOptions",
    "IKResult",
    "IKSolver",
    "IntegrationMetadata",
    "JointState",
    "MotionPrimitive",
    "MotionPrimitiveLibrary",
    "MotionResult",
    "MotionSequence",
    "MotorCalibration",
    "MotorSetupResult",
    "Pose",
    "PoseLibrary",
    "REST_POSE_NAME",
    "SO101Calibration",
    "SO101Gripper",
    "SO101KinematicModel",
    "SOArmAPI",
    "SOARM101",
    "SOARM101Config",
    "SOFTWARE_STOP_IS_CERTIFIED_EMERGENCY_STOP",
    "SavedPose",
    "SequenceLibrary",
    "SequenceRunner",
    "SequenceStep",
    "ToolAssembly",
    "Trajectory",
    "TrajectoryEntry",
    "TrajectoryLibrary",
    "__version__",
    "integration_metadata",
]
