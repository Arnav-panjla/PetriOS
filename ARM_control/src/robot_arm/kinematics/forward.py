"""Forward kinematics from the URDF joint chain. Lengths are metres."""

from __future__ import annotations

import math

from robot_arm.kinematics.geometry import ArmGeometry
from robot_arm.kinematics.transforms import (
    Vec3,
    apply_point,
    apply_vector,
    identity,
    mul,
    rot_x,
    rot_y,
    rotate_axis,
    translate,
    wrap_deg,
)


class ForwardKinematics:
    """Product of the five URDF joint transforms, then the tool point."""

    def __init__(self, geometry: ArmGeometry) -> None:
        self.geometry = geometry

    def tool_in_link_4(self, wrist_roll_rad: float) -> Vec3:
        """Tool point expressed in link_4, for the current wrist-roll angle."""
        rotated = apply_vector(rot_y(wrist_roll_rad), self.geometry.tool_in_link_5_m)
        offset = self.geometry.wrist_offset_m
        return (
            offset[0] + rotated[0],
            offset[1] + rotated[1],
            offset[2] + rotated[2],
        )

    def link_transforms(self, urdf_rad: tuple[float, ...]):
        """Return the base transform of each positioning link, in chain order."""
        if len(urdf_rad) != 5:
            raise ValueError("forward kinematics expects 5 URDF joint angles")
        frames = []
        current = identity()
        for joint, angle in zip(self.geometry.joints, urdf_rad, strict=True):
            current = mul(current, translate(joint.xyz_m))
            current = mul(current, rotate_axis(joint.axis, angle))
            frames.append(current)
        return tuple(frames)

    def tool_position(self, urdf_rad: tuple[float, ...]) -> Vec3:
        frames = self.link_transforms(urdf_rad)
        return apply_point(frames[4], self.geometry.tool_in_link_5_m)

    def tool_pitch_deg(self, urdf_rad: tuple[float, ...]) -> float:
        """Pitch of the tool direction in the arm plane, degrees.

        0 means the tool points along -Y of link_1. -90 means it points down.
        """
        direction = self._tool_direction_link1(urdf_rad)
        return wrap_deg(math.degrees(math.atan2(direction[2], -direction[1])))

    def _tool_direction_link1(self, urdf_rad: tuple[float, ...]) -> Vec3:
        tool = self.geometry.tool_in_link_5_m
        length = math.sqrt(tool[0] ** 2 + tool[1] ** 2 + tool[2] ** 2)
        if length < 1e-9:
            local = (0.0, -1.0, 0.0)
        else:
            local = (tool[0] / length, tool[1] / length, tool[2] / length)
        direction = apply_vector(rot_y(urdf_rad[4]), local)
        q_sum = urdf_rad[1] + urdf_rad[2] + urdf_rad[3]
        return apply_vector(rot_x(q_sum), direction)
