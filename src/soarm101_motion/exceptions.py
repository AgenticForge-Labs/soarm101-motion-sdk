"""Typed SDK exceptions."""


class SOARM101Error(Exception):
    """Base exception for the SDK."""


class ConfigurationError(SOARM101Error):
    pass


class RobotConnectionError(SOARM101Error):
    pass


class CalibrationError(SOARM101Error):
    pass


class CommunicationError(SOARM101Error):
    pass


class SafetyViolationError(SOARM101Error):
    pass


class InvalidJointError(SOARM101Error):
    pass


class InvalidCommandError(SOARM101Error):
    pass


class UnsupportedCapabilityError(SOARM101Error):
    pass


class MissingDependencyError(SOARM101Error):
    pass


class HardwareFaultError(SOARM101Error):
    pass
