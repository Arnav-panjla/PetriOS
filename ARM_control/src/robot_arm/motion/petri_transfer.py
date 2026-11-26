"""Level, near-rim Petri dish transfer planning. All positions are base_link metres.

This module only computes commands. It never opens a motor driver. The five-axis
arm can constrain TCP position and dish tilt; yaw is allowed to change.
"""

from __future__ import annotations

import math
import itertools
from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.optimize import least_squares

# Matches the one-slot carousel and camera stand in gazebo_stage2_gui_sync.
# Hardware fixtures must supply measured coordinates instead.
SIMULATION_SCENE = {
    # Carousel is at x=0.307591398, y=-0.100 m, yaw=+15 degrees.
    "source": {"center_m": [0.207135256, -0.073082858, 0.2400],
               "outward_xy": [-0.965925826, 0.258819045], "inside_m": 0.045},
    "destination": {"center_m": [0.060, -0.360, 0.0955],
                    "outward_xy": [0.0, 1.0], "inside_m": 0.080},
}

# The fingers close along local X. A +90 degree wrist roll makes them close
# vertically around the horizontal dish, with the upper finger above the lid
# and the lower finger through the U notch below its base.
WRIST_ROLL_RAD = math.pi / 2
DISH_NORMAL_LOCAL = np.array([-1.0, 0.0, 0.0])
MAX_DISH_TILT_DEG = 1.5

# These three J5 commands are from the recorded 2026-11-25 hardware session.
PICKUP_OPEN_DEG = 16.0
GRIP_DEG = 134.5
RELEASE_OPEN_DEG = 11.5


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
    q0 = np.clip(mod.to_q(seed), lo + 1e-7, hi - 1e-7)
    q0[4] = WRIST_ROLL_RAD
    # The fingers face in toward the dish. A weak yaw term selects that branch;
    # the hard safety constraints are position and the dish normal.
    desired_axis = np.array([-outward[0], -outward[1], 0.0])

    def residual(q_arm: np.ndarray) -> np.ndarray:
        transform, _ = mod.fk(q_arm, tcp)
        normal = transform[:3, :3] @ DISH_NORMAL_LOCAL
        axis = transform[:3, :3] @ mod.TOOL_AXIS_LOCAL
        actual = transform[:3, 3] + axis * dish_radius_m
        return np.concatenate(((actual - target) / 0.003,
                               normal[:2] / math.sin(math.radians(1)),
                               (axis[:2] - desired_axis[:2]) * 2.0,
                               (q_arm - q0) * 0.01))

    seeds = [q0]
    for joint in (0, 1, 2, 3, 4):
        for change in (-0.35, 0.35):
            trial = q0.copy()
            trial[joint] = np.clip(trial[joint] + change, lo[joint] + 1e-7, hi[joint] - 1e-7)
            seeds.append(trial)
    candidates = []
    for index, trial in enumerate(seeds):
        fit = least_squares(residual, trial, bounds=(lo, hi), max_nfev=150,
                            ftol=1e-7, xtol=1e-7, gtol=1e-7)
        transform, _ = mod.fk(fit.x, tcp)
        normal = transform[:3, :3] @ DISH_NORMAL_LOCAL
        axis = transform[:3, :3] @ mod.TOOL_AXIS_LOCAL
        position_mm = float(np.linalg.norm(transform[:3, 3] + axis * dish_radius_m - target) * 1000)
        tilt_deg = math.degrees(math.acos(float(np.clip(normal[2], -1, 1))))
        yaw_deg = math.degrees(math.acos(float(np.clip(axis @ desired_axis, -1, 1))))
        if position_mm <= 3.0 and tilt_deg <= MAX_DISH_TILT_DEG and yaw_deg <= 10.0:
            if index == 0:
                return np.asarray(mod.to_servo(fit.x), dtype=float)
            candidates.append((np.linalg.norm(fit.x - q0), position_mm, fit.x))
    if not candidates:
        raise ValueError(f"no level IK at TCP {np.round(target, 4).tolist()} "
                         "(3 mm position, 1.5 deg tilt, 10 deg approach limits)")
    q = min(candidates, key=lambda item: (item[0], item[1]))[2]
    return np.asarray(mod.to_servo(q), dtype=float)


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
            if tilt > MAX_DISH_TILT_DEG:
                raise ValueError(f"joint interpolation tilts dish {tilt:.2f} deg")


def _compact_steps(mod: Any, steps: list[TransferStep], tcp: np.ndarray,
                   grasp_offset_m: float, tolerance_m: float = 0.003) -> list[TransferStep]:
    """Keep only targets whose quintic joint interpolation follows the planned path."""
    if not steps:
        return []

    def pose(servo: tuple[float, ...] | np.ndarray, carrying: bool):
        transform = _pose(mod, np.asarray(servo)[:5], tcp)
        axis = transform[:3, :3] @ mod.TOOL_AXIS_LOCAL
        point = transform[:3, 3] + (axis * grasp_offset_m if carrying else 0.0)
        normal = transform[:3, :3] @ DISH_NORMAL_LOCAL
        return point, axis, normal, transform[2, 3]

    def follows_path(start: TransferStep, section: list[TransferStep],
                     carrying: bool) -> bool:
        end = section[-1]
        reference = [start, *section]
        positions = np.asarray([
            step.dish_center_m if carrying and step.dish_center_m is not None else step.tcp_m
            for step in reference
        ])
        axes = [pose(step.servo_deg, carrying)[1] for step in reference]
        a = np.asarray(start.servo_deg)
        b = np.asarray(end.servo_deg)
        count = max(2, math.ceil(float(np.max(np.abs(b[:5] - a[:5]))) / 0.5))
        for index in range(count + 1):
            # Quintic timing changes progress along this straight joint segment,
            # but does not change the segment's geometry.
            point, axis, normal, tcp_z = pose(a + (b - a) * (index / count), carrying)
            if tcp_z < 0.005 or normal[2] < math.cos(math.radians(MAX_DISH_TILT_DEG)):
                return False
            nearest_distance = math.inf
            nearest_axis = axes[0]
            for segment in range(len(positions) - 1):
                direction = positions[segment + 1] - positions[segment]
                fraction = float(np.clip(np.dot(point - positions[segment], direction)
                                         / (np.dot(direction, direction) + 1e-20), 0.0, 1.0))
                distance = float(np.linalg.norm(point - (positions[segment] + direction * fraction)))
                if distance < nearest_distance:
                    nearest_distance = distance
                    reference_axis = axes[segment] * (1.0 - fraction) + axes[segment + 1] * fraction
                    nearest_axis = reference_axis / np.linalg.norm(reference_axis)
            if nearest_distance > tolerance_m or axis @ nearest_axis < math.cos(math.radians(5.0)):
                return False
        return True

    compact: list[TransferStep] = []
    previous: TransferStep | None = None
    for stage, group in itertools.groupby(steps, key=lambda step: step.stage):
        section = list(group)
        if previous is None or len(section) == 1:
            compact.extend(section)
            previous = section[-1]
            continue
        carrying = section[0].dish_center_m is not None
        index = 0
        start = previous
        while index < len(section):
            for last in range(len(section) - 1, index - 1, -1):
                if follows_path(start, section[index:last + 1], carrying):
                    break
            else:
                raise ValueError(f"no safe joint interpolation through {stage}")
            compact.append(section[last])
            start = section[last]
            index = last + 1
        previous = section[-1]
    return compact


def plan_transfer(
    mod: Any, start_servo: list[float], home_servo: list[float],
    source: Station, destination: Station, *,
    rim_radius_m: float = 0.045, grasp_offset_m: float = 0.020,
    approach_m: float = 0.050,
    outside_m: float = 0.050, pickup_lift_m: float = 0.003,
    travel_lift_m: float = 0.040,
    camera_approach_m: float = 0.035,
    spacing_m: float = 0.005, close_deg: float = GRIP_DEG,
) -> list[TransferStep]:
    """Plan the complete transfer or reject it without issuing any commands."""
    if len(start_servo) != 6 or len(home_servo) != 6:
        raise ValueError("six servo angles are required")
    if not (0 < grasp_offset_m < rim_radius_m < approach_m and outside_m > rim_radius_m
            and 0 < pickup_lift_m <= 0.005 and travel_lift_m > pickup_lift_m
            and 0.025 <= camera_approach_m <= 0.150
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
        """Move a carried dish by its center while keeping it level."""
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
    opened = current.copy(); opened[5] = PICKUP_OPEN_DEG
    append("open", opened)
    cartesian("enter_grasp", source_grasp, source.outward_xy)
    closed = current.copy(); closed[5] = close_deg
    append("grip", closed, event="attach", dish=source.point(0))
    # The dish begins very close to a vertical divider. Start moving outward
    # while lifting so the rim gains lateral clearance throughout the pickup.
    pickup_escape = source_grasp.copy()
    pickup_escape[:2] += np.asarray(source.outward_xy) * 0.010
    pickup_escape[2] += pickup_lift_m
    cartesian("pickup_lift", pickup_escape,
              source.outward_xy, carrying=True)
    cartesian("extract", source_out + np.array([0.0, 0.0, pickup_lift_m]),
              source.outward_xy, carrying=True)
    cartesian("lift", source_out + np.array([0.0, 0.0, travel_lift_m]),
              source.outward_xy, carrying=True)
    # Carry above both fixtures. The dish stays level; the arm can change yaw.
    transfer_height = max(source.center_m[2], destination.center_m[2]) + travel_lift_m
    camera_approach_above = destination.point(
        camera_approach_m, transfer_height - destination.center_m[2])
    # Extraction ends on the arm side of the carousel. Keep that x clearance
    # while turning along the side of the divider: the old arm-centred arc
    # moved the dish back toward the plate and clipped its rim.
    start_transform = _pose(mod, current[:5], tcp)
    start_axis = start_transform[:3, :3] @ mod.TOOL_AXIS_LOCAL
    carry_start = start_transform[:3, 3] + start_axis * grasp_offset_m
    side_clear = np.array([carry_start[0], -0.170, transfer_height])
    for stage, target in (("carry_side", side_clear),
                          ("carry_camera", camera_approach_above)):
        transform = _pose(mod, current[:5], tcp)
        axis = transform[:3, :3] @ mod.TOOL_AXIS_LOCAL
        dish_start = transform[:3, 3] + axis * grasp_offset_m
        for point in _sample(dish_start, target, spacing_m):
            direction = -point[:2] / np.linalg.norm(point[:2])
            outward = (float(direction[0]), float(direction[1]))
            solved = _solve_level(mod, point, outward, current[:5], tcp,
                                  dish_radius_m=grasp_offset_m)
            transform = _pose(mod, solved, tcp)
            axis = transform[:3, :3] @ mod.TOOL_AXIS_LOCAL
            append(stage, np.r_[solved, current[5]],
                   dish=transform[:3, 3] + axis * grasp_offset_m)
    # Lower through the roof's front U opening at the dish centerline, then
    # slide 35 mm under its rear strip to the shelf center.
    dish_path("approach_camera", camera_approach_above, destination.outward_xy)
    dish_path("lower_camera", destination.point(camera_approach_m),
              destination.outward_xy)
    # Insert along the U-slot centerline at shelf height.
    dish_path("insert", destination.point(0), destination.outward_xy)
    opened = current.copy(); opened[5] = RELEASE_OPEN_DEG
    append("release", opened, event="detach", dish=destination.point(0))
    # Withdraw at shelf height, then use one low clearance pose outside the
    # front opening before returning home. The arm need not climb to carry
    # height after the dish has been released.
    release_tcp = _pose(mod, current[:5], tcp)[:3, 3]
    retreat_tcp = release_tcp.copy()
    retreat_tcp[:2] += np.asarray(destination.outward_xy) * camera_approach_m
    cartesian("retreat", retreat_tcp, destination.outward_xy)
    clear_tcp = retreat_tcp.copy()
    clear_tcp[:2] += np.asarray(destination.outward_xy) * 0.055
    clear_tcp[2] += 0.055
    clear_solution = _solve_level(mod, clear_tcp, destination.outward_xy,
                                  current[:5], tcp)
    append("clear_camera", np.r_[clear_solution, current[5]], level=False)
    append("home", home, level=False)
    return _compact_steps(mod, steps, tcp, grasp_offset_m)
