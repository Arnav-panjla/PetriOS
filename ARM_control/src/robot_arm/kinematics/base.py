"""Shared kinematic types. Hardware code does not import these."""

from __future__ import annotations

from dataclasses import dataclass

from robot_arm.kinematics.transforms import Vec3


@dataclass(frozen=True)
class Pose:
    """Commanded tool pose. There is no measured pose."""

    position_m: Vec3
    tool_pitch_deg: float
    r_mm: float
    theta_deg: float
    z_mm: float

    @property
    def position_mm(self) -> Vec3:
        return (
            self.position_m[0] * 1000.0,
            self.position_m[1] * 1000.0,
            self.position_m[2] * 1000.0,
        )


@dataclass(frozen=True)
class JointConfiguration:
    """One IK answer, in URDF radians and servo degrees."""

    urdf_rad: tuple[float, float, float, float, float]
    servo_deg: tuple[float, float, float, float, float, float]
    elbow: str
    residual_mm: float
    residual_pitch_deg: float
    source: str
    warning: str = ""
