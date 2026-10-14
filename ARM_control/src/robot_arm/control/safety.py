"""Startup and limit checks. This does not sense the arm."""

from __future__ import annotations

import logging
import math

from robot_arm.hardware.servo import TRAVEL_DEG
from robot_arm.utils.errors import JointLimitError, NotReadyError

logger = logging.getLogger(__name__)


class SafetyManager:
    """Blocks PWM until startup is confirmed, and keeps angles inside 0..180."""

    def __init__(self, require_confirmation: bool) -> None:
        self.require_confirmation = require_confirmation
        self.pwm_allowed = False

    def authorize_pwm(self) -> None:
        self.pwm_allowed = True

    def ensure_pwm_allowed(self) -> None:
        if not self.pwm_allowed:
            raise NotReadyError(
                "PWM is blocked until startup is confirmed. "
                "The physical pose is unknown, so motion will not be interpolated "
                "from it."
            )

    def clamp_angle(self, degrees: float) -> float:
        if not math.isfinite(degrees):
            raise JointLimitError(f"angle is not finite: {degrees}")
        if degrees < 0.0 or degrees > TRAVEL_DEG:
            logger.warning(
                "angle %.3f deg is outside 0..%.0f and will be clamped",
                degrees,
                TRAVEL_DEG,
            )
            return min(TRAVEL_DEG, max(0.0, degrees))
        return degrees

    def check_index(self, index: int, count: int) -> None:
        if not 0 <= index < count:
            raise JointLimitError(f"joint must be 0..{count - 1}, got {index}")
