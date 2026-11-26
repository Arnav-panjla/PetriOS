"""Run the level Petri transfer directly through Gazebo ros2_control."""

from __future__ import annotations

import itertools
import math
import re
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .direct_config import direct_mapping, load_home, load_ik, repository_root
from .dish_event import send_dish_event
from .mapping import ARM_JOINT_NAMES, GRIPPER_JOINT_NAMES


def _trajectory(names: list[str], targets: list[list[float]], durations: list[float]) -> JointTrajectory:
    message = JointTrajectory()
    message.joint_names = names
    elapsed = 0.0
    for target, duration in zip(targets, durations):
        elapsed += duration
        point = JointTrajectoryPoint()
        point.positions = target
        # ros2_control uses a quintic spline when both endpoint velocity and
        # acceleration are supplied. Stop smoothly at each selected key pose.
        point.velocities = [0.0] * len(names)
        point.accelerations = [0.0] * len(names)
        nanoseconds = round(elapsed * 1_000_000_000)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(nanoseconds, 1_000_000_000)
        message.points.append(point)
    return message


class DirectTransfer(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_direct_petri_transfer")
        self.declare_parameter("speed", 1.5)
        self.speed = float(self.get_parameter("speed").value)
        if not 0.2 <= self.speed <= 3.0:
            raise ValueError("speed must be between 0.2 and 3.0")
        self.arm_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, "/gripper_controller/joint_trajectory", 10)
        self.positions: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._joint_state, 20)

    def _joint_state(self, message: JointState) -> None:
        self.positions.update(zip(message.name, message.position))

    def _spin_until(self, condition, timeout_s: float, reason: str) -> None:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if condition():
                return
        raise RuntimeError(f"timed out waiting for {reason}")

    def _settle_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _settled(self, arm: list[float], grip: list[float], *, precise: bool) -> bool:
        expected = dict(zip(ARM_JOINT_NAMES + GRIPPER_JOINT_NAMES, arm + grip))
        arm_tolerance = 0.025 if precise else 0.045
        return all(
            name in self.positions
            and abs(self.positions[name] - value) <= (arm_tolerance if name in ARM_JOINT_NAMES else 0.055)
            for name, value in expected.items()
        )

    def _event(self, kind: str) -> None:
        send_dish_event(kind)
        self.get_logger().info(f"Gazebo {kind} confirmed")

    def _dish_pose(self) -> tuple[float, float, float] | None:
        result = subprocess.run(
            ["ign", "topic", "-t", "/world/amr_stage1_world/dynamic_pose/info", "-e", "-n", "1"],
            capture_output=True, text=True, timeout=5, check=False,
        )
        match = re.search(
            r'name: "petri_dish_1".*?position \{\s*x: ([^\s]+)\s*y: ([^\s]+)\s*z: ([^\s]+)',
            result.stdout, re.S,
        )
        return tuple(float(value) for value in match.groups()) if match else None

    def run(self) -> None:
        ik = load_ik()
        sys.path.insert(0, str(repository_root() / "ARM_control/src"))
        import numpy as np
        from robot_arm.motion.petri_transfer import SIMULATION_SCENE, Station, _solve_level, plan_transfer

        mapping = direct_mapping(ik)
        home = load_home()
        steps = plan_transfer(
            ik, home, home,
            Station.from_dict(SIMULATION_SCENE["source"]),
            Station.from_dict(SIMULATION_SCENE["destination"]),
        )
        self.get_logger().info(f"planned {len(steps)} level waypoints; speed={self.speed:g}x")
        self._spin_until(
            lambda: self.arm_pub.get_subscription_count() > 0
            and self.gripper_pub.get_subscription_count() > 0
            and all(name in self.positions for name in ARM_JOINT_NAMES + GRIPPER_JOINT_NAMES),
            45.0, "Gazebo controllers and joint states",
        )
        current_arm, current_grip = mapping.all_positions(home)
        release_arm_override = None
        for stage, grouped in itertools.groupby(steps, key=lambda step: step.stage):
            segment = list(grouped)
            targets = [mapping.all_positions(step.servo_deg) for step in segment]
            if stage == "release" and release_arm_override is not None:
                targets = [(release_arm_override, grip) for _, grip in targets]
            durations = []
            previous_arm, previous_grip = current_arm, current_grip
            for index, (arm, grip) in enumerate(targets):
                delta = max(abs(a - b) for a, b in zip(previous_arm + previous_grip, arm + grip))
                duration = max(0.10 / self.speed, delta / (0.85 * self.speed))
                if stage == "insert" and index >= len(targets) - 12:
                    duration = max(duration, 0.25 / math.sqrt(self.speed))
                if stage == "enter_grasp":
                    duration = max(duration, 0.30)
                if stage == "pickup_lift":
                    duration = max(duration, 0.50)
                if stage == "extract" and index < 8:
                    duration = max(duration, 0.60)
                if stage in ("carry_side", "carry_camera"):
                    duration = max(duration, 0.45)
                if stage == "retreat" and index < 15:
                    duration = max(duration, 1.00)
                if stage == "clear_camera":
                    duration = max(duration, 0.70)
                if stage == "home":
                    duration = max(duration, 2.50)
                if stage in ("grip", "release"):
                    duration = max(duration, 0.8)
                durations.append(duration)
                previous_arm, previous_grip = arm, grip
            arm_targets = [arm for arm, _ in targets]
            grip_targets = [grip for _, grip in targets]
            if stage == "enter_grasp":
                for waypoint, ((arm, grip), duration) in enumerate(zip(targets, durations), start=1):
                    self.arm_pub.publish(_trajectory(ARM_JOINT_NAMES, [arm], [duration]))
                    self.gripper_pub.publish(_trajectory(GRIPPER_JOINT_NAMES, [grip], [duration]))
                    self._spin_until(
                        lambda arm=arm, grip=grip: self._settled(arm, grip, precise=True),
                        12.0, f"{stage} waypoint {waypoint}/{len(targets)}",
                    )
                current_arm, current_grip = targets[-1]
                self.get_logger().info(
                    f"{stage}: {len(segment)} settled points, {sum(durations):.1f} s commanded"
                )
                continue
            self.arm_pub.publish(_trajectory(ARM_JOINT_NAMES, arm_targets, durations))
            self.gripper_pub.publish(_trajectory(GRIPPER_JOINT_NAMES, grip_targets, durations))
            current_arm, current_grip = targets[-1]
            duration = sum(durations)
            self.get_logger().info(f"{stage}: {len(segment)} points, {duration:.1f} s simulated")
            precise = stage == "insert" or any(step.event for step in segment)
            self._spin_until(
                lambda: self._settled(current_arm, current_grip, precise=precise),
                max(20.0, duration * 6.0 + 12.0), f"{stage} joints to settle",
            )
            if stage in ("pickup_lift", "extract"):
                self.get_logger().info(f"dish after {stage}: {self._dish_pose()}")
            if stage == "insert":
                self._settle_for(0.5)
                # DetachableJoint preserves any small pickup offset. Measure the
                # actual dish and translate it horizontally before release. The
                # camera roof precludes any vertical correction inside the stand.
                destination = Station.from_dict(SIMULATION_SCENE["destination"])
                for _ in range(2):
                    pose = self._dish_pose()
                    if pose is None:
                        raise RuntimeError("Petri dish pose unavailable for alignment")
                    error_xy = np.asarray(destination.center_m[:2]) - np.asarray(pose[:2])
                    error_mm = float(np.linalg.norm(error_xy) * 1000)
                    if error_mm <= 1.5:
                        break
                    if error_mm > 12.0:
                        raise RuntimeError(f"dish is {error_mm:.1f} mm off center before release")
                    servo = mapping.servo_positions(current_arm)
                    transform, _ = ik.fk(ik.to_q(servo), ik.TCP_ESTIMATE)
                    target_tcp = transform[:3, 3].copy()
                    target_tcp[:2] += error_xy
                    corrected_servo = _solve_level(
                        ik, target_tcp, destination.outward_xy,
                        np.asarray(servo), np.asarray(ik.TCP_ESTIMATE),
                    )
                    corrected_arm = mapping.arm_positions(corrected_servo)
                    adjustment_s = max(0.35 / self.speed,
                                       max(abs(a - b) for a, b in zip(current_arm, corrected_arm)) / (0.4 * self.speed))
                    self.arm_pub.publish(_trajectory(ARM_JOINT_NAMES, [corrected_arm], [adjustment_s]))
                    self._spin_until(
                        lambda: self._settled(corrected_arm, current_grip, precise=True),
                        15.0, "dish alignment",
                    )
                    self._settle_for(0.3)
                    current_arm = corrected_arm
                    self.get_logger().info(f"aligned dish horizontally by {error_mm:.1f} mm")
                release_arm_override = current_arm
                self.get_logger().info(f"dish before release: {self._dish_pose()}")
            for step in segment:
                if step.event:
                    if step.event == "attach":
                        pose = self._dish_pose()
                        self.get_logger().info(f"dish before attach: {pose}")
                        source = SIMULATION_SCENE["source"]["center_m"]
                        if pose is None or math.dist(pose, source) > 0.005:
                            raise RuntimeError("dish is not seated in the carousel before pickup")
                    self._event(step.event)
                    if step.event == "attach":
                        self._settle_for(0.2)
                        self.get_logger().info(f"dish after attach: {self._dish_pose()}")
                    if step.event == "detach":
                        self._settle_for(0.5)
                        self.get_logger().info(f"dish after release: {self._dish_pose()}")
            if stage == "retreat":
                self.get_logger().info(f"dish after retreat: {self._dish_pose()}")
        pose = self._dish_pose()
        if pose is None:
            raise RuntimeError("Petri dish pose unavailable after transfer")
        target = SIMULATION_SCENE["destination"]["center_m"]
        error_mm = math.dist(pose, target) * 1000.0
        self.get_logger().info(f"complete: dish center={tuple(round(v, 4) for v in pose)}, error={error_mm:.1f} mm")
        if error_mm > 10.0:
            raise RuntimeError(f"dish ended {error_mm:.1f} mm from camera shelf center")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DirectTransfer()
    try:
        node.run()
    except (KeyboardInterrupt, RuntimeError, ValueError) as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1) from exc
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
