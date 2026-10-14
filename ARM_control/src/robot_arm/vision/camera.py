"""Camera extension point. No camera calibration is stored here."""

from __future__ import annotations

from typing import Protocol


class Camera(Protocol):
    """Future image source for dish detection."""

    def capture(self) -> object:
        """Return one frame. Not implemented."""


class ExtrinsicCalibration(Protocol):
    """Future camera-to-robot transform. Not measured."""

    def pixel_to_robot(self, u: float, v: float) -> tuple[float, float, float]:
        """Map a pixel into the robot frame. Not implemented."""


class UnavailableCamera:
    """Placeholder so vision code can be added without touching PWM control."""

    def capture(self) -> object:
        raise NotImplementedError("No camera backend is configured.")
