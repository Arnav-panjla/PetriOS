"""Planar 2R shoulder/elbow solve, plus a numerical fallback."""

from __future__ import annotations

import math

from robot_arm.kinematics.forward import ForwardKinematics
from robot_arm.kinematics.geometry import ArmGeometry
from robot_arm.kinematics.transforms import (
    Vec3,
    apply_vector,
    rot_x,
    rot_y,
    wrap_pi,
)


def pitch_sum(tool_pitch_rad: float, wrist_roll_rad: float, geometry: ArmGeometry) -> float:
    """q2 + q3 + q4 that puts the tool at the requested arm-plane pitch."""
    tool = geometry.tool_in_link_5_m
    length = math.sqrt(tool[0] ** 2 + tool[1] ** 2 + tool[2] ** 2)
    if length < 1e-9:
        local = (0.0, -1.0, 0.0)
    else:
        local = (tool[0] / length, tool[1] / length, tool[2] / length)
    direction = apply_vector(rot_y(wrist_roll_rad), local)
    alpha = math.atan2(direction[1], direction[2])
    return wrap_pi(alpha + math.pi / 2.0 - tool_pitch_rad)


def solve_elbow(
    target_yz: tuple[float, float],
    upper_m: float,
    forearm_m: float,
) -> list[tuple[float, float]]:
    """Return (q2, q3) pairs that put joint 4 at target_yz relative to the shoulder.

    q3 = 0 keeps the forearm along -Y while the upper arm is along +Z.
    """
    py, pz = target_yz
    reach_sq = py * py + pz * pz
    denominator = 2.0 * upper_m * forearm_m
    if denominator <= 0.0:
        return []
    sine = (upper_m * upper_m + forearm_m * forearm_m - reach_sq) / denominator
    if abs(sine) > 1.0000001:
        return []
    sine = min(1.0, max(-1.0, sine))
    answers = []
    primary = math.asin(sine)
    for q3 in (primary, math.pi - primary):
        y3 = -forearm_m * math.cos(q3)
        z3 = upper_m - forearm_m * math.sin(q3)
        q2 = wrap_pi(math.atan2(pz, py) - math.atan2(z3, y3))
        answers.append((q2, wrap_pi(q3)))
    return answers


def wrist_target_yz(tcp_link1: Vec3, pitch_sum_rad: float, tool_in_link4: Vec3) -> tuple[float, float]:
    """YZ of joint 4 in link_1 that puts the tool on the target at this pitch sum."""
    shifted = apply_vector(rot_x(pitch_sum_rad), tool_in_link4)
    return (tcp_link1[1] - shifted[1], tcp_link1[2] - shifted[2])


def numerical_refine(
    forward: ForwardKinematics,
    seed: tuple[float, float, float, float, float],
    tcp_link1: Vec3,
    tool_pitch_rad: float,
    *,
    steps: int = 40,
) -> tuple[float, float, float, float, float]:
    """Damped least squares on q2, q3 and q4. q1 and q5 stay at the seed."""
    q = list(seed)
    lam = 1e-3
    for _ in range(steps):
        error = _residual(forward, tuple(q), tcp_link1, tool_pitch_rad)
        if sum(item * item for item in error) < 1e-16:
            break
        columns = []
        for index in (1, 2, 3):
            bumped = q[:]
            bumped[index] += 1e-6
            bumped_error = _residual(forward, tuple(bumped), tcp_link1, tool_pitch_rad)
            columns.append(
                tuple((bumped_error[row] - error[row]) / 1e-6 for row in range(4))
            )
        normal = [[0.0, 0.0, 0.0] for _ in range(3)]
        rhs = [0.0, 0.0, 0.0]
        for col in range(3):
            for row in range(4):
                rhs[col] += columns[col][row] * error[row]
            for other in range(3):
                normal[col][other] = sum(
                    columns[col][row] * columns[other][row] for row in range(4)
                )
            normal[col][col] += lam
        try:
            step = _solve_3(normal, rhs)
        except ZeroDivisionError:
            break
        for index, delta in zip((1, 2, 3), step, strict=True):
            q[index] = wrap_pi(q[index] - delta)
    return (q[0], q[1], q[2], q[3], q[4])


def _residual(
    forward: ForwardKinematics,
    urdf_rad: tuple[float, ...],
    tcp_link1: Vec3,
    tool_pitch_rad: float,
) -> tuple[float, float, float, float]:
    world = forward.tool_position(urdf_rad)
    link1 = _invert_apply(forward.link_transforms(urdf_rad)[0], world)
    pitch = math.radians(forward.tool_pitch_deg(urdf_rad))
    return (
        link1[0] - tcp_link1[0],
        link1[1] - tcp_link1[1],
        link1[2] - tcp_link1[2],
        math.atan2(math.sin(pitch - tool_pitch_rad), math.cos(pitch - tool_pitch_rad)),
    )


def _invert_apply(transform, point: Vec3) -> Vec3:
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


def _solve_3(matrix: list[list[float]], vector: list[float]) -> list[float]:
    aug = [row[:] + [value] for row, value in zip(matrix, vector, strict=True)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(aug[row][col]))
        aug[col], aug[pivot] = aug[pivot], aug[col]
        scale = aug[col][col]
        if abs(scale) < 1e-12:
            raise ZeroDivisionError
        for index in range(col, 4):
            aug[col][index] /= scale
        for row in range(3):
            if row == col:
                continue
            factor = aug[row][col]
            for index in range(col, 4):
                aug[row][index] -= factor * aug[col][index]
    return [aug[row][3] for row in range(3)]
