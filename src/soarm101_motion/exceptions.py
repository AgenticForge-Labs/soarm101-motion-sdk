"""Exception hierarchy for the SO-ARM101 SDK."""


class SOARM101Error(Exception):
    """Base exception for all SDK errors."""


class ConfigurationError(SOARM101Error):
    """The supplied configuration is invalid."""


class RobotConnectionError(SOARM101Error):
    """The SDK could not establish or maintain the robot connection."""


class CalibrationError(SOARM101Error):
    """Calibration is missing, invalid, or unsafe to use."""


class CalibrationCancelledError(CalibrationError):
    """A calibration sweep was cancelled and its motor settings restored."""


class CommunicationError(SOARM101Error):
    """A Feetech packet or serial communication operation failed."""


class SafetyViolationError(SOARM101Error):
    """A motion command violated configured limits."""


class InvalidJointError(SafetyViolationError):
    """A command referenced an unknown joint."""


class InvalidCommandError(SOARM101Error):
    """A command is malformed or cannot be executed in the current state."""


class UnsupportedCapabilityError(SOARM101Error):
    """The active backend or tool does not implement the requested capability."""


class MissingDependencyError(SOARM101Error):
    """An optional runtime dependency is unavailable."""


class HardwareFaultError(SOARM101Error):
    """One or more motors reported a hardware fault."""


class IKError(SOARM101Error):
    """Inverse kinematics failed to find an acceptable solution."""


class MotionCancelledError(SOARM101Error):
    """A motion was cancelled before completion."""


class MotionTimeoutError(SOARM101Error):
    """A motion did not settle at its target before the configured timeout."""
