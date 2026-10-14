"""One joint: clamp, convert, and command a pulse."""

from __future__ import annotations

from robot_arm.control.safety import SafetyManager
from robot_arm.hardware.driver import PWMDriver
from robot_arm.hardware.servo import Servo


class JointController:
    """Commanded state for a single servo. There is no measured position."""

    def __init__(
        self,
        servo: Servo,
        driver: PWMDriver,
        safety: SafetyManager,
    ) -> None:
        self.servo = servo
        self._driver = driver
        self._safety = safety

    @property
    def commanded_deg(self) -> float:
        return self.servo.commanded_deg

    def command(self, degrees: float) -> float:
        """Write one pulse. Returns the angle that was actually commanded."""
        self._safety.ensure_pwm_allowed()
        clamped = self._safety.clamp_angle(degrees)
        pulse = self.servo.pulse_us(clamped)
        self._driver.set_pulse_us(self.servo.channel, pulse)
        self.servo.set_commanded(clamped)
        return clamped
