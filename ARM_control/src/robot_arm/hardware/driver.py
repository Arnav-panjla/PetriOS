"""PWM outputs. The PCA9685 path is open-loop; the mock records commands."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Protocol

from robot_arm.hardware.servo import TRAVEL_DEG, us_to_deg

logger = logging.getLogger(__name__)


class PWMDriver(Protocol):
    """Something that can emit a servo pulse on a PCA9685 channel."""

    def set_pulse_us(self, channel: int, pulse_us: float) -> None:
        """Command one channel. Does not report where the horn actually is."""

    def close(self) -> None:
        """Release the bus if this driver owns one."""


@dataclass(frozen=True)
class PWMWrite:
    time_s: float
    channel: int
    pulse_us: float


class MockDriver:
    """Records pulses in memory so tests can run without a Pi."""

    def __init__(self, clock=time.monotonic) -> None:
        self.writes: list[PWMWrite] = []
        self._clock = clock
        self.closed = False

    def set_pulse_us(self, channel: int, pulse_us: float) -> None:
        self.writes.append(PWMWrite(self._clock(), channel, pulse_us))

    def close(self) -> None:
        self.closed = True


class PCA9685Driver:
    """ServoKit writer using the same angle map as the original script.

    Each channel's configured min/max pulse is 0..180 degrees. This does not
    read the servo back, and channels updated in one software tick are not
    hardware-synchronized beyond the chip's free-running PWM counter.
    """

    def __init__(self, config: object) -> None:
        from robot_arm.config import RobotConfig

        if not isinstance(config, RobotConfig):
            raise TypeError("PCA9685Driver requires a RobotConfig")
        try:
            from board import SCL, SDA
            import busio
            from adafruit_servokit import ServoKit
        except ImportError as exc:
            raise SystemExit(
                "Missing hardware library. From ARM_control/robotic-arm:\n"
                "  pip install -e '.[hardware]'\n"
                "On Pi OS Trixie that build needs swig, python3-dev, "
                "build-essential, and liblgpio-dev.\n"
                f"Import error: {exc}"
            ) from exc

        i2c = busio.I2C(SCL, SDA)
        self._i2c = i2c
        self._kit = ServoKit(
            channels=16,
            i2c=i2c,
            address=config.hardware.address,
            frequency=config.hardware.pwm_frequency_hz,
        )
        self._ranges: dict[int, tuple[int, int]] = {}
        for joint in config.joints:
            servo = self._kit.servo[joint.channel]
            servo.actuation_range = int(TRAVEL_DEG)
            servo.set_pulse_width_range(joint.min_us, joint.max_us)
            self._ranges[joint.channel] = (joint.min_us, joint.max_us)
        # The ESP32 sketch waits 500 ms after pwm.begin() before the first pulses.
        time.sleep(0.5)
        logger.info(
            "PCA9685 ready on I2C bus %s address 0x%02X at %s Hz",
            config.hardware.i2c_bus,
            config.hardware.address,
            config.hardware.pwm_frequency_hz,
        )

    def set_pulse_us(self, channel: int, pulse_us: float) -> None:
        min_us, max_us = self._ranges[channel]
        angle = us_to_deg(min_us, max_us, pulse_us)
        angle = min(TRAVEL_DEG, max(0.0, angle))
        self._kit.servo[channel].angle = angle

    def close(self) -> None:
        deinit = getattr(self._i2c, "deinit", None)
        if deinit is not None:
            deinit()
