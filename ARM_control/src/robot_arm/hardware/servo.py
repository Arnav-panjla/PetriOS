"""Pulse calibration for one servo. Angles here are commanded, never measured."""

from __future__ import annotations

from dataclasses import dataclass

from robot_arm.utils.errors import JointLimitError

TRAVEL_DEG = 180.0


def us_to_deg(min_us: float, max_us: float, pulse_us: float) -> float:
    """Map a pulse inside [min_us, max_us] onto 0..180 degrees."""
    return (pulse_us - min_us) * TRAVEL_DEG / (max_us - min_us)


def deg_to_us(min_us: float, max_us: float, degrees: float) -> float:
    """Inverse of us_to_deg. 0 degrees is min_us and 180 degrees is max_us."""
    return min_us + (max_us - min_us) * (degrees / TRAVEL_DEG)


@dataclass(frozen=True)
class ServoCalibration:
    """Pulse limits copied from the existing arm sketch."""

    index: int
    channel: int
    min_us: int
    max_us: int
    home_us: int
    max_velocity_deg_s: float
    max_acceleration_deg_s2: float

    @property
    def home_deg(self) -> float:
        return us_to_deg(self.min_us, self.max_us, self.home_us)


class Servo:
    """One servo's calibration and the last angle this process commanded."""

    def __init__(self, calibration: ServoCalibration) -> None:
        self.calibration = calibration
        self.commanded_deg = calibration.home_deg

    @property
    def index(self) -> int:
        return self.calibration.index

    @property
    def channel(self) -> int:
        return self.calibration.channel

    def pulse_us(self, degrees: float) -> float:
        return deg_to_us(
            self.calibration.min_us,
            self.calibration.max_us,
            degrees,
        )

    def commanded_pulse_us(self) -> int:
        return int(round(self.pulse_us(self.commanded_deg)))

    def set_commanded(self, degrees: float) -> None:
        if degrees < 0.0 or degrees > TRAVEL_DEG:
            raise JointLimitError(
                f"joint {self.index} angle {degrees} is outside 0..{TRAVEL_DEG:.0f}"
            )
        self.commanded_deg = degrees
