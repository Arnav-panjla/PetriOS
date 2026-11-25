"""Keep the Gazebo-only launch calibration aligned with the arm IK model."""

import numpy as np

from arm_gazebo_bridge.direct_config import (
    DIRECT_SIGN, DIRECT_TCP_M, DIRECT_ZERO_DEG, direct_mapping, load_home, load_ik,
)


def test_direct_launch_uses_ik_calibration_and_saved_home():
    ik = load_ik()
    assert np.allclose(DIRECT_ZERO_DEG, ik.URDF_ZERO_SERVO)
    assert np.allclose(DIRECT_SIGN, ik.SERVO_SIGN)
    assert np.allclose(DIRECT_TCP_M, ik.TCP_ESTIMATE)
    assert np.allclose(direct_mapping().arm_positions(load_home()),
                       direct_mapping(ik).arm_positions(load_home()))
