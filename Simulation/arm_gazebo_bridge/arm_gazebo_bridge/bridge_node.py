"""ROS 2 node that mirrors commanded web-GUI servo angles into Gazebo."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .mapping import (
    ARM_JOINT_NAMES,
    GRIPPER_JOINT_NAMES,
    GuiConnectionError,
    ServoMapping,
    request_json,
    status_angles,
)


class GuiToGazeboBridge(Node):
    def __init__(self) -> None:
        super().__init__("gui_to_gazebo_bridge")
        self.declare_parameter("gui_url", "http://127.0.0.1:8000")
        self.declare_parameter("poll_hz", 20.0)
        self.declare_parameter("http_timeout_s", 2.0)
        self.declare_parameter("trajectory_duration_s", 0.1)
        self.declare_parameter("arm_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("gripper_topic", "/gripper_controller/joint_trajectory")

        self.gui_url = str(self.get_parameter("gui_url").value)
        self.timeout = float(self.get_parameter("http_timeout_s").value)
        self.duration = float(self.get_parameter("trajectory_duration_s").value)
        poll_hz = float(self.get_parameter("poll_hz").value)
        if poll_hz <= 0.0 or self.timeout <= 0.0 or self.duration <= 0.0:
            raise ValueError("poll_hz, http_timeout_s and trajectory_duration_s must be positive")

        self.arm_pub = self.create_publisher(
            JointTrajectory, str(self.get_parameter("arm_topic").value), 10
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory, str(self.get_parameter("gripper_topic").value), 10
        )
        self.mapping: ServoMapping | None = None
        self.pending: tuple[list[float], list[float]] | None = None
        self.connected = False
        self.last_error = ""
        self.create_timer(1.0 / poll_hz, self.poll)
        self.get_logger().info(f"mirroring {self.gui_url} at {poll_hz:g} Hz")

    def poll(self) -> None:
        try:
            if self.mapping is None:
                self.mapping = ServoMapping.from_model(
                    request_json(self.gui_url, "/api/model", self.timeout)
                )
            status = request_json(self.gui_url, "/api/status", self.timeout)
            self.pending = self.mapping.all_positions(status_angles(status))
            if not self.connected:
                self.get_logger().info("GUI connected; calibrated mapping loaded")
            self.connected = True
            self.last_error = ""
        except (GuiConnectionError, ValueError) as exc:
            message = str(exc)
            if message != self.last_error:
                self.get_logger().error(f"GUI unavailable; holding last Gazebo command: {message}")
                self.last_error = message
            self.connected = False
            return

        if self.arm_pub.get_subscription_count() == 0 or self.gripper_pub.get_subscription_count() == 0:
            return
        arm, gripper = self.pending
        self.arm_pub.publish(self._trajectory(ARM_JOINT_NAMES, arm))
        self.gripper_pub.publish(self._trajectory(GRIPPER_JOINT_NAMES, gripper))

    def _trajectory(self, names: list[str], positions: list[float]) -> JointTrajectory:
        message = JointTrajectory()
        # A zero stamp means "start now" in joint_trajectory_controller. The
        # GUI bridge uses wall time for HTTP polling while Gazebo uses /clock,
        # so copying the bridge clock here would place goals far in the future.
        message.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = positions
        seconds = int(self.duration)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = int(round((self.duration - seconds) * 1_000_000_000))
        message.points = [point]
        return message


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GuiToGazeboBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
