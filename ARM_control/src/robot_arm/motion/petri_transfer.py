"""Level, near-rim Petri dish transfer planning. All positions are base_link metres.

This module only computes commands. It never opens a motor driver. The five-axis
arm can constrain TCP position and dish tilt; yaw is allowed to change.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.optimize import least_squares

# Matches the one-slot carousel and camera stand in gazebo_stage2_gui_sync.
# Hardware fixtures must supply measured coordinates instead.
SIMULATION_SCENE = {
    # Both fixtures are shifted 50 mm radially toward the arm from the prior scene.
    "source": {"center_m": [0.244659, -0.244659, 0.2400],
               "outward_xy": [-0.70710678, 0.70710678], "inside_m": 0.045},
    "destination": {"center_m": [0.0, -0.390, 0.0925],
                    "outward_xy": [0.0, 1.0], "inside_m": 0.080},
}

# The fingers close along local X. A +90 degree wrist roll makes them close
# vertically around the horizontal dish, with the upper finger above the lid
# and the lower finger through the U notch below its base.
WRIST_ROLL_RAD = math.pi / 2
DISH_NORMAL_LOCAL = np.array([-1.0, 0.0, 0.0])


@dataclass(frozen=True)
class Station:
    center_m: tuple[float, float, float]
    outward_xy: tuple[float, float]
    inside_m: float = 0.045

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "Station":
        center = tuple(float(x) for x in value["center_m"])
        outward = tuple(float(x) for x in value["outward_xy"])
        inside = float(value.get("inside_m", 0.045))
        if len(center) != 3 or len(outward) != 2 or not all(
            math.isfinite(x) for x in (*center, *outward, inside)
        ):
            raise ValueError("station requires finite center_m[3], outward_xy[2], inside_m")
        length = math.hypot(*outward)
        if length < 1e-9 or inside < 0:
            raise ValueError("station outward direction must be nonzero and inside_m >= 0")
        return cls(center, (outward[0] / length, outward[1] / length), inside)

    def point(self, center_offset_m: float, z_offset_m: float = 0.0) -> np.ndarray:
        return np.array(
            [self.center_m[0] + self.outward_xy[0] * center_offset_m,
             self.center_m[1] + self.outward_xy[1] * center_offset_m,
             self.center_m[2] + z_offset_m], dtype=float
        )


@dataclass(frozen=True)
class TransferStep:
    stage: str
    servo_deg: tuple[float, ...]
    tcp_m: tuple[float, float, float]
    dish_center_m: tuple[float, float, float] | None
    event: str | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "stage": self.stage,
            "servo_deg": list(self.servo_deg),
            "tcp_m": list(self.tcp_m),
            "dish_center_m": list(self.dish_center_m) if self.dish_center_m else None,
            "event": self.event,
        }


def _sample(start: np.ndarray, end: np.ndarray, spacing_m: float) -> list[np.ndarray]:
    count = max(1, math.ceil(float(np.linalg.norm(end - start)) / spacing_m))
    return [start + (end - start) * (index / count) for index in range(1, count + 1)]


def _pose(mod: Any, servo: np.ndarray, tcp: np.ndarray) -> np.ndarray:
    return mod.fk(mod.to_q(servo), tcp)[0]


def _solve_level(
    mod: Any, target: np.ndarray, outward: tuple[float, float],
    seed: np.ndarray, tcp: np.ndarray, dish_radius_m: float = 0.0,
) -> np.ndarray:
    lo, hi = mod.limits_q()
    if not lo[4] < WRIST_ROLL_RAD < hi[4]:
        raise ValueError("90 degree wrist roll is outside the configured soft limits")
    q0 = np.clip(mod.to_q(seed)[:4], lo[:4] + 1e-7, hi[:4] - 1e-7)
    # The fingers face in toward the dish. A weak yaw term selects that branch;
    # the hard safety constraints are position and the dish normal.
    desired_axis = np.array([-outward[0], -outward[1], 0.0])

    def residual(q_arm: np.ndarray) -> np.ndarray:
        transform, _ = mod.fk(np.r_[q_arm, WRIST_ROLL_RAD], tcp)
        normal = transform[:3, :3] @ DISH_NORMAL_LOCAL
        axis = transform[:3, :3] @ mod.TOOL_AXIS_LOCAL
        actual = transform[:3, 3] + axis * dish_radius_m
        return np.concatenate(((actual - target) / 0.003,
                               normal[:2] / math.sin(math.radians(1)),
                               (axis[:2] - desired_axis[:2]) * 2.0,
                               (q_arm - q0) * 0.01))

    seeds = [q0]
    for joint in (0, 1, 2, 3):
        for change in (-0.35, 0.35):
            trial = q0.copy()
            trial[joint] = np.clip(trial[joint] + change, lo[joint] + 1e-7, hi[joint] - 1e-7)
            seeds.append(trial)
    candidates = []
    for trial in seeds:
        fit = least_squares(residual, trial, bounds=(lo[:4], hi[:4]), max_nfev=150,
                            ftol=1e-7, xtol=1e-7, gtol=1e-7)
        transform, _ = mod.fk(np.r_[fit.x, WRIST_ROLL_RAD], tcp)
        normal = transform[:3, :3] @ DISH_NORMAL_LOCAL
        axis = transform[:3, :3] @ mod.TOOL_AXIS_LOCAL
        position_mm = float(np.linalg.norm(transform[:3, 3] + axis * dish_radius_m - target) * 1000)
        tilt_deg = math.degrees(math.acos(float(np.clip(normal[2], -1, 1))))
        yaw_deg = math.degrees(math.acos(float(np.clip(axis @ desired_axis, -1, 1))))
        if position_mm <= 2.0 and tilt_deg <= 1.0 and yaw_deg <= 10.0:
            candidates.append((np.linalg.norm(fit.x - q0), position_mm, fit.x))
    if not candidates:
        raise ValueError(f"no level IK at TCP {np.round(target, 4).tolist()} "
                         "(2 mm position, 1 deg tilt, 10 deg approach limits)")
    q = min(candidates, key=lambda item: (item[0], item[1]))[2]
    return np.asarray(mod.to_servo(np.r_[q, WRIST_ROLL_RAD]), dtype=float)


def _check_joint_segment(mod: Any, a: np.ndarray, b: np.ndarray, tcp: np.ndarray,
                         *, level: bool, floor_m: float = 0.005) -> None:
    # ArmController's quintic applies one common blend to every joint. Thus
    # samples of the straight joint segment cover the executed joint path.
    count = max(1, math.ceil(float(np.max(np.abs(b[:5] - a[:5]))) / 0.5))
    for index in range(count + 1):
        servo = a[:5] + (b[:5] - a[:5]) * (index / count)
        transform = _pose(mod, servo, tcp)
        if transform[2, 3] < floor_m:
            raise ValueError("planned TCP passes below the 5 mm floor guard")
        if level:
            normal = transform[:3, :3] @ DISH_NORMAL_LOCAL
            tilt = math.degrees(math.acos(float(np.clip(normal[2], -1, 1))))
            if tilt > 1.0:
                raise ValueError(f"joint interpolation tilts dish {tilt:.2f} deg")


def plan_transfer(
    mod: Any, start_servo: list[float], home_servo: list[float],
    source: Station, destination: Station, *,
    rim_radius_m: float = 0.045, grasp_offset_m: float = 0.020,
    approach_m: float = 0.050,
    outside_m: float = 0.060, pickup_lift_m: float = 0.003,
    travel_lift_m: float = 0.040,
    release_clearance_m: float = 0.005,
    spacing_m: float = 0.005, close_deg: float = 95.0,
) -> list[TransferStep]:
    """Plan the complete transfer or reject it without issuing any commands."""
    if len(start_servo) != 6 or len(home_servo) != 6:
        raise ValueError("six servo angles are required")
    if not (0 < grasp_offset_m < rim_radius_m < approach_m and outside_m > rim_radius_m
            and 0 < pickup_lift_m <= 0.005 and travel_lift_m > pickup_lift_m
            and 0 < release_clearance_m <= 0.007
            and 0 < spacing_m <= 0.005
            and 10.0 < close_deg <= 167.5):
        raise ValueError("invalid dish geometry or motion settings")
    tcp = np.asarray(mod.TCP_ESTIMATE, dtype=float)
    current = np.asarray(start_servo, dtype=float)
    home = np.asarray(home_servo, dtype=float)
    steps: list[TransferStep] = []

    def append(stage: str, servo: np.ndarray, *, level: bool = True,
               event: str | None = None, dish: np.ndarray | None = None) -> None:
        nonlocal current
        _check_joint_segment(mod, current, servo, tcp, level=level)
        transform = _pose(mod, servo[:5], tcp)
        steps.append(TransferStep(stage, tuple(float(v) for v in servo),
                                  tuple(float(v) for v in transform[:3, 3]),
                                  tuple(float(v) for v in dish) if dish is not None else None,
                                  event))
        current = servo.copy()

    def cartesian(stage: str, target: np.ndarray, outward: tuple[float, float],
                  *, carrying: bool = False) -> None:
        start = _pose(mod, current[:5], tcp)[:3, 3]
        for point in _sample(start, target, spacing_m):
            solved = _solve_level(mod, point, outward, current[:5], tcp)
            next_servo = np.r_[solved, current[5]]
            transform = _pose(mod, solved, tcp)
            axis = transform[:3, :3] @ mod.TOOL_AXIS_LOCAL
            dish = transform[:3, 3] + axis * grasp_offset_m if carrying else None
            append(stage, next_servo, dish=dish)

    def dish_path(stage: str, target_center: np.ndarray,
                  outward: tuple[float, float]) -> None:
        """Move a carried dish by its center, including vertical roof pass-through."""
        transform = _pose(mod, current[:5], tcp)
        axis = transform[:3, :3] @ mod.TOOL_AXIS_LOCAL
        dish_start = transform[:3, 3] + axis * grasp_offset_m
        for dish_point in _sample(dish_start, target_center, spacing_m):
            solved = _solve_level(mod, dish_point, outward, current[:5], tcp,
                                  dish_radius_m=grasp_offset_m)
            transform = _pose(mod, solved, tcp)
            axis = transform[:3, :3] @ mod.TOOL_AXIS_LOCAL
            actual_center = transform[:3, 3] + axis * grasp_offset_m
            append(stage, np.r_[solved, current[5]], dish=actual_center)

    source_pre = source.point(approach_m)
    source_grasp = source.point(grasp_offset_m)
    source_out = source.point(grasp_offset_m + source.inside_m + outside_m)

    pre_solution = _solve_level(mod, source_pre, source.outward_xy, current[:5], tcp)
    append("pregrasp", np.r_[pre_solution, current[5]], level=False)
    opened = current.copy(); opened[5] = 10.0
    append("open", opened)
    cartesian("enter_grasp", source_grasp, source.outward_xy)
    closed = current.copy(); closed[5] = close_deg
    append("grip", closed, event="attach", dish=source.point(0))
    # Lift clear of the orange holder before moving horizontally. The attach
    # event is issued only after the gripper has reached its lightly closed pose.
    cartesian("pickup_lift", source_grasp + np.array([0.0, 0.0, pickup_lift_m]),
              source.outward_xy, carrying=True)
    cartesian("extract", source_out + np.array([0.0, 0.0, pickup_lift_m]),
              source.outward_xy, carrying=True)
    cartesian("lift", source_out + np.array([0.0, 0.0, travel_lift_m]),
              source.outward_xy, carrying=True)
    # Carry above both fixtures. The dish stays level; the arm can change yaw.
    transfer_height = max(source.center_m[2], destination.center_m[2]) + travel_lift_m
    destination_above = destination.point(0.0, transfer_height - destination.center_m[2])
    # Move farther away from the carousel before turning toward the camera.
    # Using the larger arm-centred radius here points back toward the carousel
    # (whose centre lies beyond the source along the same radial line) and can
    # drive the dish through a vertical divider.
    source_xy = source_out[:2]
    destination_xy = destination.center_m[:2]
    source_radius = float(np.linalg.norm(source_xy))
    destination_radius = float(np.linalg.norm(destination_xy))
    arc_radius = min(source_radius, destination_radius) - 0.025
    if arc_radius <= 0.15:
        raise ValueError("fixtures leave no safe arm-centred carry arc")
    source_angle = math.atan2(source_xy[1], source_xy[0])
    destination_angle = math.atan2(destination_xy[1], destination_xy[0])
    source_via = np.array([arc_radius * math.cos(source_angle),
                           arc_radius * math.sin(source_angle), transfer_height])
    dest_via = np.array([arc_radius * math.cos(destination_angle),
                         arc_radius * math.sin(destination_angle), transfer_height])
    cartesian("carry_out", source_via, source.outward_xy, carrying=True)
    delta_angle = (destination_angle - source_angle + math.pi) % (2 * math.pi) - math.pi
    arc_count = max(1, math.ceil(abs(delta_angle) * arc_radius / spacing_m))
    for index in range(1, arc_count + 1):
        angle = source_angle + delta_angle * index / arc_count
        point = np.array([arc_radius * math.cos(angle),
                          arc_radius * math.sin(angle), transfer_height])
        outward = (-math.cos(angle), -math.sin(angle))
        solved = _solve_level(mod, point, outward, current[:5], tcp)
        transform = _pose(mod, solved, tcp)
        axis = transform[:3, :3] @ mod.TOOL_AXIS_LOCAL
        append("carry_arc", np.r_[solved, current[5]],
               dish=transform[:3, 3] + axis * grasp_offset_m)
    # The dish approaches above the camera, then passes down through the
    # collision-disabled upper plate into the inner U slot.
    dish_path("approach_camera", destination_above, destination.outward_xy)
    dish_path("insert", destination.point(0), destination.outward_xy)
    opened = current.copy(); opened[5] = 10.0
    append("release", opened, event="detach", dish=destination.point(0))
    # Clear the dish, then retract vertically back through the upper plate.
    release_tcp = _pose(mod, current[:5], tcp)[:3, 3]
    clearance_tcp = release_tcp + np.array([0.0, 0.0, release_clearance_m])
    cartesian("clear_dish", clearance_tcp, destination.outward_xy)
    retreat_tcp = release_tcp.copy()
    retreat_tcp[2] = transfer_height
    cartesian("retreat", retreat_tcp, destination.outward_xy)
    append("home", home, level=False)
    return steps
