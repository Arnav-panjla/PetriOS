"""Hardware-free checks for the GUI's cylindrical IK modes."""

import math

import pytest

from robot_arm.interfaces.web_gui import IKBridge, from_cylindrical, to_cylindrical

AXIS = (0.0, -0.029745)


def test_theta_zero_points_along_minus_y() -> None:
    r, theta, z = to_cylindrical((0.0, AXIS[1] - 0.2, 0.1), AXIS)
    assert r == pytest.approx(200.0)
    assert theta == pytest.approx(0.0, abs=1e-9)
    assert z == pytest.approx(100.0)


def test_positive_theta_is_counter_clockwise_from_above() -> None:
    # +X is 90 degrees counter-clockwise from -Y seen from +z.
    _r, theta, _z = to_cylindrical((0.1, AXIS[1], 0.0), AXIS)
    assert theta == pytest.approx(90.0)


@pytest.mark.parametrize("r,theta,z", [(150.0, 0.0, 90.0), (80.0, 35.0, 20.0), (200.0, -60.0, 130.0)])
def test_round_trip(r: float, theta: float, z: float) -> None:
    xyz = from_cylindrical(r, theta, z, AXIS)
    back = to_cylindrical(xyz, AXIS)
    assert back == pytest.approx((r, theta, z), abs=1e-6)


def _bridge() -> IKBridge:
    pytest.importorskip("numpy")
    pytest.importorskip("scipy")
    ik = IKBridge()
    if not ik.ok:
        pytest.skip(ik.error)
    return ik


def test_cylindrical_modes_agree_with_cartesian() -> None:
    ik = _bridge()
    current = [float(v) for v in ik.mod.START_SERVO] + [167.5]
    zero = ik.solve(current, {"mode": "cyl_rel", "x": 0, "y": 0, "z": 0})
    assert zero["err_mm"] < 3.0
    assert zero["target_cyl"] == pytest.approx(zero["cur_cyl"], abs=1e-6)

    # Absolute cylindrical at the current TCP is the same target as dr=dtheta=dz=0.
    r, theta, z = zero["cur_cyl"]
    absolute = ik.solve(current, {"mode": "cyl_abs", "x": r, "y": theta, "z": z})
    assert absolute["target_mm"] == pytest.approx(zero["target_mm"], abs=1e-6)

    # A small dz moves the target straight up and leaves r and theta alone.
    up = ik.solve(current, {"mode": "cyl_rel", "x": 0, "y": 0, "z": 10})
    assert up["target_cyl"][0] == pytest.approx(r, abs=1e-6)
    assert up["target_cyl"][1] == pytest.approx(theta, abs=1e-6)
    assert up["target_cyl"][2] == pytest.approx(z + 10, abs=1e-6)


def test_negative_radius_is_rejected() -> None:
    ik = _bridge()
    current = [float(v) for v in ik.mod.START_SERVO] + [167.5]
    r = ik.solve(current, {"mode": "cyl_rel", "x": 0, "y": 0, "z": 0})["cur_cyl"][0]
    with pytest.raises(ValueError, match="r must be"):
        ik.solve(current, {"mode": "cyl_rel", "x": -(r + 50), "y": 0, "z": 0})
    with pytest.raises(ValueError, match="unknown IK mode"):
        ik.solve(current, {"mode": "nope", "x": 0, "y": 0, "z": 0})
