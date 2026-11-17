import math

import pytest

from arm_gazebo_bridge.mapping import ServoMapping, status_angles


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


def test_saved_home_conversion_and_round_trip() -> None:
    mapping = ServoMapping.from_model(MODEL)
    servo = [88.5, 36.0, 56.5, 151.0, 116.25]
    q = mapping.arm_positions(servo)
    assert [math.degrees(value) for value in q] == pytest.approx([-1.5, -3.0, 1.5, 84.5, 0.0])
    assert mapping.servo_positions(q) == pytest.approx(servo)


def test_gripper_endpoints_are_mirrored_and_linked() -> None:
    mapping = ServoMapping.from_model(MODEL)
    opened = mapping.gripper_positions(10.0)
    closed = mapping.gripper_positions(167.5)
    assert opened[1] == pytest.approx(-opened[0])
    assert closed[1] == pytest.approx(-closed[0])
    assert opened[2] == pytest.approx(0.81 * opened[0])
    assert closed[3] == pytest.approx(-0.81 * closed[0])
    assert closed[0] > opened[0]
    assert all(-0.8 <= value <= 0.8 for value in opened + closed)


def test_status_angles_requires_six_joints() -> None:
    status = {"joints": [{"home_deg": value} for value in [1, 2, 3, 4, 5, 6]]}
    assert status_angles(status, "home_deg") == [1, 2, 3, 4, 5, 6]
