"""Small rigid transforms. Angles are radians. Lengths are metres."""

from __future__ import annotations

import math

Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]
Mat4 = tuple[tuple[float, float, float, float], ...]


def identity() -> Mat4:
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def mul(a: Mat4, b: Mat4) -> Mat4:
    rows = []
    for i in range(4):
        rows.append(
            tuple(sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4))
        )
    return tuple(rows)  # type: ignore[return-value]


def translate(xyz: Vec3) -> Mat4:
    return (
        (1.0, 0.0, 0.0, xyz[0]),
        (0.0, 1.0, 0.0, xyz[1]),
        (0.0, 0.0, 1.0, xyz[2]),
        (0.0, 0.0, 0.0, 1.0),
    )


def rotate_axis(axis: Vec3, angle: float) -> Mat4:
    """Rodrigues rotation about a unit axis."""
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    one = 1.0 - c
    rotation = (
        (c + x * x * one, x * y * one - z * s, x * z * one + y * s),
        (y * x * one + z * s, c + y * y * one, y * z * one - x * s),
        (z * x * one - y * s, z * y * one + x * s, c + z * z * one),
    )
    return (
        (rotation[0][0], rotation[0][1], rotation[0][2], 0.0),
        (rotation[1][0], rotation[1][1], rotation[1][2], 0.0),
        (rotation[2][0], rotation[2][1], rotation[2][2], 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def rotation_of(transform: Mat4) -> Mat3:
    return (
        (transform[0][0], transform[0][1], transform[0][2]),
        (transform[1][0], transform[1][1], transform[1][2]),
        (transform[2][0], transform[2][1], transform[2][2]),
    )


def apply_point(transform: Mat4, point: Vec3) -> Vec3:
    x, y, z = point
    return (
        transform[0][0] * x + transform[0][1] * y + transform[0][2] * z + transform[0][3],
        transform[1][0] * x + transform[1][1] * y + transform[1][2] * z + transform[1][3],
        transform[2][0] * x + transform[2][1] * y + transform[2][2] * z + transform[2][3],
    )


def apply_vector(rotation: Mat3, vector: Vec3) -> Vec3:
    x, y, z = vector
    return (
        rotation[0][0] * x + rotation[0][1] * y + rotation[0][2] * z,
        rotation[1][0] * x + rotation[1][1] * y + rotation[1][2] * z,
        rotation[2][0] * x + rotation[2][1] * y + rotation[2][2] * z,
    )


def rot_x(angle: float) -> Mat3:
    c = math.cos(angle)
    s = math.sin(angle)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def rot_y(angle: float) -> Mat3:
    c = math.cos(angle)
    s = math.sin(angle)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def deg(radians: float) -> float:
    return math.degrees(radians)


def rad(degrees: float) -> float:
    return math.radians(degrees)


def wrap_deg(angle: float) -> float:
    return deg(wrap_pi(rad(angle)))
