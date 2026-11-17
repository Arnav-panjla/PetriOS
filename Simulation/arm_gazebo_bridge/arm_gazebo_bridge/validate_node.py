"""Simulation-only end-to-end TCP validation for GUI -> bridge -> Gazebo."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from .mapping import (
    ARM_JOINT_NAMES,
    GuiConnectionError,
    ServoMapping,
    request_json,
    status_angles,
)


def _rotate(axis: list[float], angle: float, vector: list[float]) -> list[float]:
    x, y, z = axis
    vx, vy, vz = vector
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return [
        (t*x*x+c)*vx + (t*x*y-s*z)*vy + (t*x*z+s*y)*vz,
        (t*x*y+s*z)*vx + (t*y*y+c)*vy + (t*y*z-s*x)*vz,
        (t*x*z-s*y)*vx + (t*y*z+s*x)*vy + (t*z*z+c)*vz,
    ]


def expected_tcp(model: dict, q: list[float]) -> list[float]:
    rotation = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    position = [0.0, 0.0, 0.0]

    def world(vector: list[float]) -> list[float]:
        return [sum(rotation[row][column] * vector[column] for column in range(3)) for row in range(3)]

    for joint, angle in zip(model["joints"], q):
        offset = world([float(value) for value in joint["xyz"]])
        position = [a + b for a, b in zip(position, offset)]
        axis_world = world([float(value) for value in joint["axis"]])
        columns = [[rotation[row][column] for row in range(3)] for column in range(3)]
        turned = [_rotate(axis_world, angle, column) for column in columns]
        rotation = [[turned[column][row] for column in range(3)] for row in range(3)]
    tcp_world = world([float(value) for value in model["tcp"]])
    return [a + b for a, b in zip(position, tcp_world)]


class Validator(Node):
    def __init__(self) -> None:
        super().__init__("validate_arm_kinematics")
        self.declare_parameter("gui_url", "http://127.0.0.1:8000")
        self.declare_parameter("http_timeout_s", 2.0)
        self.declare_parameter("settle_timeout_s", 12.0)
        self.declare_parameter("joint_tolerance_rad", 0.002)
        self.declare_parameter("tcp_tolerance_mm", 1.0)
        self.gui_url = str(self.get_parameter("gui_url").value)
        self.http_timeout = float(self.get_parameter("http_timeout_s").value)
        self.settle_timeout = float(self.get_parameter("settle_timeout_s").value)
        self.joint_tolerance = float(self.get_parameter("joint_tolerance_rad").value)
        self.tcp_tolerance_mm = float(self.get_parameter("tcp_tolerance_mm").value)
        self.positions: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._joint_state, 20)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _joint_state(self, message: JointState) -> None:
        self.positions.update(zip(message.name, message.position))

    def wait_for_pose(self, target: list[float]) -> list[float]:
        deadline = time.monotonic() + self.settle_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(name in self.positions for name in ARM_JOINT_NAMES):
                error = max(abs(self.positions[name] - value) for name, value in zip(ARM_JOINT_NAMES, target))
                if error <= self.joint_tolerance:
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            "base_link", "tcp_link", Time(), timeout=Duration(seconds=0.2)
                        )
                    except TransformException:
                        continue
                    translation = transform.transform.translation
                    return [translation.x, translation.y, translation.z]
        raise RuntimeError("Gazebo joints/TCP did not settle before timeout")

    def run(self) -> int:
        status = request_json(self.gui_url, "/api/status", self.http_timeout)
        if not status.get("simulation_only"):
            raise RuntimeError("refusing validation: GUI is not running with --simulation-only")
        model = request_json(self.gui_url, "/api/model", self.http_timeout)
        mapping = ServoMapping.from_model(model)
        home = status_angles(status, "home_deg")
        previous_soften = status.get("soften_ms")
        zero = list(mapping.zero_deg) + [mapping.gripper_open_deg]
        sample_1 = [100.0, 47.0, 68.0, 90.0, 106.0, 80.0]
        # Keep the TCP above the world plane; configurations below z=0 are
        # intentionally excluded because collision response prevents a pure
        # kinematic comparison.
        sample_2 = [95.0, 45.0, 65.0, 100.0, 110.0, 140.0]
        cases = [("saved_home", home), ("urdf_zero", zero), ("sample_1", sample_1), ("sample_2", sample_2)]
        worst = 0.0
        try:
            # Simulation-only validation should not spend tens of seconds in
            # the physical arm's conservative quintic planner.
            request_json(self.gui_url, "/api/soften", self.http_timeout, {"ms": 0.0})
            for name, servo in cases:
                request_json(
                    self.gui_url,
                    "/api/move",
                    self.http_timeout,
                    {"angles": {str(index): value for index, value in enumerate(servo)}},
                )
                q = mapping.arm_positions(servo)
                observed = self.wait_for_pose(q)
                expected = expected_tcp(model, q)
                mismatch = math.dist(expected, observed) * 1000.0
                worst = max(worst, mismatch)
                self.get_logger().info(
                    f"{name}: expected_mm={[round(v*1000, 3) for v in expected]} "
                    f"gazebo_mm={[round(v*1000, 3) for v in observed]} mismatch={mismatch:.3f} mm"
                )
        finally:
            try:
                request_json(self.gui_url, "/api/home", self.http_timeout, {})
                request_json(
                    self.gui_url,
                    "/api/soften",
                    self.http_timeout,
                    {"ms": previous_soften},
                )
            except GuiConnectionError as exc:
                self.get_logger().error(f"could not restore home: {exc}")
        self.get_logger().info(f"maximum TCP mismatch: {worst:.3f} mm")
        return 0 if worst <= self.tcp_tolerance_mm else 1


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Validator()
    code = 1
    try:
        code = node.run()
    except (GuiConnectionError, RuntimeError, KeyError, ValueError) as exc:
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)
