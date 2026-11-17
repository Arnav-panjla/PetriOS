import rclpy

from arm_gazebo_bridge import bridge_node
from arm_gazebo_bridge.mapping import GuiConnectionError


MODEL = {
    "servo_mapping": {
        "zero_deg": [90.0, 39.0, 58.0, 66.5, 116.25],
        "sign": [1.0, 1.0, -1.0, 1.0, -1.0],
    },
    "grip": {"closed_deg": 167.5, "open_deg": 10.0, "max_width_m": 0.035},
    "gripper": {
        "left": {"xyz": [-0.005981, -0.068816, -0.0015]},
        "right": {"xyz": [0.025679, -0.068611, 0.0015]},
    },
}
STATUS = {
    "joints": [
        {"commanded_deg": value}
        for value in [88.5, 36.0, 56.5, 151.0, 116.25, 41.5]
    ]
}


class FakePublisher:
    def __init__(self, subscriptions=0):
        self.subscriptions = subscriptions
        self.messages = []

    def get_subscription_count(self):
        return self.subscriptions

    def publish(self, message):
        self.messages.append(message)


def test_poll_waits_for_both_controllers_and_holds_on_disconnect(monkeypatch):
    rclpy.init()
    node = bridge_node.GuiToGazeboBridge()
    arm = FakePublisher()
    gripper = FakePublisher()
    node.arm_pub = arm
    node.gripper_pub = gripper

    def connected(_base, path, _timeout):
        return MODEL if path == "/api/model" else STATUS

    monkeypatch.setattr(bridge_node, "request_json", connected)
    node.poll()
    assert node.pending is not None
    assert arm.messages == []
    assert gripper.messages == []

    arm.subscriptions = gripper.subscriptions = 1
    node.poll()
    assert len(arm.messages) == len(gripper.messages) == 1
    assert arm.messages[0].header.stamp.sec == 0
    assert arm.messages[0].header.stamp.nanosec == 0
    assert arm.messages[0].joint_names == ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]

    monkeypatch.setattr(
        bridge_node,
        "request_json",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(GuiConnectionError("offline")),
    )
    node.poll()
    assert len(arm.messages) == len(gripper.messages) == 1
    node.destroy_node()
    rclpy.shutdown()
