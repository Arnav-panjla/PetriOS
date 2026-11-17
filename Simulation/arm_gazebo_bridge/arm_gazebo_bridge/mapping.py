"""Pure-Python calibration and HTTP helpers shared by launch, bridge and tests."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

ARM_JOINT_NAMES = [f"joint_{index}" for index in range(1, 6)]
GRIPPER_JOINT_NAMES = [
    "left_finger_joint",
    "right_finger_joint",
    "left_gripper_link_joint",
    "right_gripper_link_joint",
]


class GuiConnectionError(RuntimeError):
    """The GUI endpoint was unavailable or returned malformed data."""


def _url(base: str, path: str) -> str:
    return base.rstrip("/") + path


def request_json(
    base: str,
    path: str,
    timeout: float,
    body: dict[str, Any] | None = None,
) -> dict[str, Any]:
    data = None if body is None else json.dumps(body).encode("utf-8")
    request = Request(
        _url(base, path),
        data=data,
        headers={"Content-Type": "application/json"},
        method="GET" if body is None else "POST",
    )
    try:
        with urlopen(request, timeout=timeout) as response:
            result = json.load(response)
    except (HTTPError, URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
        raise GuiConnectionError(f"{request.full_url}: {exc}") from exc
    if not isinstance(result, dict):
        raise GuiConnectionError(f"{request.full_url}: expected a JSON object")
    if result.get("error"):
        raise GuiConnectionError(f"{request.full_url}: {result['error']}")
    return result


@dataclass(frozen=True)
class ServoMapping:
    zero_deg: tuple[float, ...]
    sign: tuple[float, ...]
    gripper_closed_deg: float
    gripper_open_deg: float
    gripper_max_width_m: float
    finger_pivot_separation_m: float
    finger_length_m: float = 0.045
    linkage_ratio: float = 0.81

    @classmethod
    def from_model(cls, model: dict[str, Any]) -> "ServoMapping":
        try:
            servo = model["servo_mapping"]
            grip = model["grip"]
            gripper = model["gripper"]
            zero = tuple(float(value) for value in servo["zero_deg"])
            sign = tuple(float(value) for value in servo["sign"])
            left_x = float(gripper["left"]["xyz"][0])
            right_x = float(gripper["right"]["xyz"][0])
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            raise GuiConnectionError(f"invalid /api/model payload: {exc}") from exc
        if len(zero) != 5 or len(sign) != 5 or any(value not in (-1.0, 1.0) for value in sign):
            raise GuiConnectionError("/api/model servo mapping must contain five zeros and five +/-1 signs")
        return cls(
            zero_deg=zero,
            sign=sign,
            gripper_closed_deg=float(grip["closed_deg"]),
            gripper_open_deg=float(grip["open_deg"]),
            gripper_max_width_m=float(grip["max_width_m"]),
            finger_pivot_separation_m=right_x - left_x,
        )

    def arm_positions(self, servo_deg: list[float] | tuple[float, ...]) -> list[float]:
        if len(servo_deg) < 5:
            raise ValueError("expected at least five servo angles")
        return [
            math.radians(sign * (float(angle) - zero))
            for angle, zero, sign in zip(servo_deg[:5], self.zero_deg, self.sign)
        ]

    def servo_positions(self, q_rad: list[float] | tuple[float, ...]) -> list[float]:
        if len(q_rad) != 5:
            raise ValueError("expected five URDF joint positions")
        return [zero + sign * math.degrees(q) for q, zero, sign in zip(q_rad, self.zero_deg, self.sign)]

    def gripper_positions(self, servo_deg: float) -> list[float]:
        span = self.gripper_closed_deg - self.gripper_open_deg
        fraction_open = (self.gripper_closed_deg - float(servo_deg)) / span
        fraction_open = min(1.0, max(0.0, fraction_open))
        denominator = 2.0 * self.finger_length_m
        closed = math.asin(min(1.0, self.finger_pivot_separation_m / denominator))
        opened = math.asin(
            min(1.0, max(-1.0, (self.finger_pivot_separation_m - self.gripper_max_width_m) / denominator))
        )
        left = closed + fraction_open * (opened - closed)
        values = [left, -left, self.linkage_ratio * left, -self.linkage_ratio * left]
        return [min(0.8, max(-0.8, value)) for value in values]

    def all_positions(self, servo_deg: list[float] | tuple[float, ...]) -> tuple[list[float], list[float]]:
        if len(servo_deg) != 6:
            raise ValueError("expected six servo angles")
        return self.arm_positions(servo_deg), self.gripper_positions(float(servo_deg[5]))


def status_angles(status: dict[str, Any], field: str = "commanded_deg") -> list[float]:
    try:
        joints = status["joints"]
        angles = [float(joint[field]) for joint in joints]
    except (KeyError, TypeError, ValueError) as exc:
        raise GuiConnectionError(f"invalid /api/status payload: {exc}") from exc
    if len(angles) != 6:
        raise GuiConnectionError("/api/status must contain six joints")
    return angles
