"""Errors raised by arm control. They are not hardware faults by themselves."""


class RobotArmError(Exception):
    """Base error for this package."""


class ConfigError(RobotArmError):
    """YAML configuration is missing or inconsistent."""


class JointLimitError(RobotArmError):
    """A joint index or angle is outside the calibrated range."""


class NotReadyError(RobotArmError):
    """PWM was requested before startup authorization."""
