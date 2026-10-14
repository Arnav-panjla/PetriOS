"""Cylindrical coordinates and the servo-facing IK wrapper."""

from __future__ import annotations

import math

from robot_arm.calibration.coordinate_calibration import CoordinateCalibration
from robot_arm.calibration.joint_calibration import JointCalibration
from robot_arm.kinematics.base import JointConfiguration, Pose
from robot_arm.kinematics.forward import ForwardKinematics
from robot_arm.kinematics.geometry import ArmGeometry
from robot_arm.kinematics.inverse import (
    numerical_refine,
    pitch_sum,
    solve_elbow,
    wrist_target_yz,
)
from robot_arm.kinematics.transforms import Vec3, deg, rad, wrap_deg, wrap_pi
from robot_arm.utils.errors import JointLimitError, UnreachableError


class CylindricalKinematics:
    """FK and cylindrical IK. Servo pulses are not touched here."""

    def __init__(
        self,
        geometry: ArmGeometry,
        joints: JointCalibration,
        coordinates: CoordinateCalibration,
    ) -> None:
        self.geometry = geometry
        self.joints = joints
        self.coordinates = coordinates
        self.forward = ForwardKinematics(geometry)

    def fk(self, servo_deg: tuple[float, ...]) -> Pose:
        if len(servo_deg) < 5:
            raise ValueError("fk expects at least the five positioning servos")
        urdf = self.joints.to_urdf(servo_deg[:5])
        position = self._corrected_position(self.forward.tool_position(urdf))
        pitch = wrap_deg(
            self.forward.tool_pitch_deg(urdf) + self.coordinates.tool_pitch_offset_deg
        )
        radius, theta, height = self._base_cylindrical(position, urdf[0])
        return Pose(
            position_m=position,
            tool_pitch_deg=pitch,
            r_mm=radius * 1000.0 + self.coordinates.r_offset_mm,
            theta_deg=wrap_deg(
                self.coordinates.theta_direction * deg(theta)
                + self.coordinates.theta_zero_offset_deg
            ),
            z_mm=height * 1000.0 + self.coordinates.z_offset_mm,
        )

    def ik(
        self,
        r_mm: float,
        theta_deg: float,
        z_mm: float,
        tool_pitch_deg: float,
        *,
        current_servo_deg: tuple[float, float, float, float, float, float],
        elbow: str | None = None,
    ) -> JointConfiguration:
        geometric_theta = rad(
            self.coordinates.theta_direction
            * (theta_deg - self.coordinates.theta_zero_offset_deg)
        )
        radius = (r_mm - self.coordinates.r_offset_mm) / 1000.0
        height = (z_mm - self.coordinates.z_offset_mm) / 1000.0
        pitch = rad(tool_pitch_deg - self.coordinates.tool_pitch_offset_deg)
        q1 = wrap_pi(geometric_theta)
        q5 = self.joints.to_urdf(current_servo_deg[:5])[4]
        tool_in_link4 = self.forward.tool_in_link_4(q5)
        q_sum = pitch_sum(pitch, q5, self.geometry)
        shoulder = self.geometry.shoulder_xyz
        # Rotation about X does not change the local X of the tool, so the
        # arm cannot aim that coordinate. It is fixed by the URDF offsets.
        tcp = (shoulder[0] + tool_in_link4[0], -radius, height)
        target_yz = wrist_target_yz(tcp, q_sum, tool_in_link4)
        relative = (target_yz[0] - shoulder[1], target_yz[1] - shoulder[2])
        warning = "" if self.geometry.planar else self.geometry.planar_note
        branches = [] if not self.geometry.planar else solve_elbow(
            relative,
            self.geometry.upper_arm_m,
            self.geometry.forearm_m,
        )
        candidates = []
        for index, (q2, q3) in enumerate(branches):
            q4 = wrap_pi(q_sum - q2 - q3)
            label = "down" if index == 0 else "up"
            if elbow is not None and label != elbow:
                continue
            candidates.append((q1, q2, q3, q4, q5, label, "analytic"))
        if not candidates and elbow is None:
            seed = self.joints.to_urdf(current_servo_deg[:5])
            refined = numerical_refine(
                self.forward,
                (q1, seed[1], seed[2], seed[3], q5),
                tcp,
                pitch,
            )
            candidates.append(
                (
                    refined[0],
                    refined[1],
                    refined[2],
                    refined[3],
                    refined[4],
                    "numerical",
                    "numerical",
                )
            )
        scored: list[JointConfiguration] = []
        for q1_i, q2, q3, q4, q5_i, label, source in candidates:
            try:
                servos = self.joints.to_servo(
                    (q1_i, q2, q3, q4, q5_i),
                    current_servo_deg,
                )
            except (ValueError, JointLimitError):
                continue
            scored.append(
                self._score(
                    (q1_i, q2, q3, q4, q5_i),
                    servos,
                    tcp,
                    pitch,
                    label,
                    source,
                    warning,
                )
            )
        valid = [
            item
            for item in scored
            if item.residual_mm <= self.coordinates.position_tolerance_mm
            and abs(item.residual_pitch_deg) <= self.coordinates.pitch_tolerance_deg
        ]
        if not valid:
            detail = (
                f"r={r_mm:g} mm, theta={theta_deg:g} deg, z={z_mm:g} mm, "
                f"tool_pitch={tool_pitch_deg:g} deg"
            )
            if scored:
                best = min(scored, key=lambda item: item.residual_mm)
                detail += (
                    f"; closest residual {best.residual_mm:.2f} mm, "
                    f"pitch {best.residual_pitch_deg:.2f} deg"
                )
            raise UnreachableError(detail)
        return min(
            valid,
            key=lambda item: sum(
                (left - right) ** 2
                for left, right in zip(item.servo_deg[:4], current_servo_deg[:4], strict=True)
            ),
        )

    def _score(
        self,
        urdf_rad: tuple[float, float, float, float, float],
        servo_deg: tuple[float, float, float, float, float, float],
        tcp_link1: Vec3,
        tool_pitch_rad: float,
        elbow: str,
        source: str,
        warning: str,
    ) -> JointConfiguration:
        world = self.forward.tool_position(urdf_rad)
        link1 = _invert_point(self.forward.link_transforms(urdf_rad)[0], world)
        residual_m = math.sqrt(
            sum((left - right) ** 2 for left, right in zip(link1, tcp_link1, strict=True))
        )
        measured = math.radians(self.forward.tool_pitch_deg(urdf_rad))
        pitch_error = math.degrees(
            math.atan2(
                math.sin(measured - tool_pitch_rad),
                math.cos(measured - tool_pitch_rad),
            )
        )
        return JointConfiguration(
            urdf_rad=urdf_rad,
            servo_deg=servo_deg,
            elbow=elbow,
            residual_mm=residual_m * 1000.0,
            residual_pitch_deg=pitch_error,
            source=source,
            warning=warning,
        )

    def _corrected_position(self, position: Vec3) -> Vec3:
        correction = self.coordinates.position_correction_m
        return (
            position[0] + correction[0],
            position[1] + correction[1],
            position[2] + correction[2],
        )

    def _base_cylindrical(
        self,
        position: Vec3,
        q1: float,
    ) -> tuple[float, float, float]:
        """r, theta, z about the joint_1 axis. Positive r is -Y at theta 0."""
        origin = self.geometry.joints[0].xyz_m
        relative = (
            position[0] - origin[0],
            position[1] - origin[1],
            position[2] - origin[2],
        )
        cosine = math.cos(-q1)
        sine = math.sin(-q1)
        y = sine * relative[0] + cosine * relative[1]
        return -y, wrap_pi(q1), relative[2]


def _invert_point(transform, point: Vec3) -> Vec3:
    from robot_arm.kinematics.transforms import apply_vector

    rotation = (
        (transform[0][0], transform[1][0], transform[2][0]),
        (transform[0][1], transform[1][1], transform[2][1]),
        (transform[0][2], transform[1][2], transform[2][2]),
    )
    shifted = (
        point[0] - transform[0][3],
        point[1] - transform[1][3],
        point[2] - transform[2][3],
    )
    return apply_vector(rotation, shifted)
