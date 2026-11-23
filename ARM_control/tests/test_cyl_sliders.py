"""r, h, theta, pitch sliders and the Keep-level toggle (hardware-free)."""

import threading

import pytest

pytest.importorskip("numpy")
pytest.importorskip("scipy")

from robot_arm.interfaces.web_gui import ArmService, IKBridge  # noqa: E402

SAFE = [88.5, 69.0, 56.5, 152.0, 116.25, 41.4]


@pytest.fixture(scope="module")
def ik():
    bridge = IKBridge()
    if not bridge.ok:
        pytest.skip("IK module unavailable")
    return bridge


class FakeJoint:
    def __init__(self, deg):
        self.commanded_deg = deg


class FakeArm:
    ready = True
    soften_ms = None

    def __init__(self, pose):
        self._joints = [FakeJoint(d) for d in pose]


def service(ik, pose=SAFE):
    svc = ArmService.__new__(ArmService)
    svc.arm = FakeArm(pose)
    svc.ik = ik
    svc.floor_enabled = True
    svc.floor_mm = 5.0
    svc.level_enabled = False
    svc.level_roll_deg = 28.0
    svc.home_deg = list(SAFE)
    svc._lock = threading.Lock()
    svc._cyl_lock = threading.Lock()
    svc._pending = {}
    svc._wake = threading.Event()
    return svc


@pytest.fixture(scope="module")
def pose(ik):
    """A pose far enough out that a level tool is reachable (tool pitch -20)."""
    _, theta, _ = ik.cylindrical(SAFE)
    return ik.solve_cyl(SAFE, 300.0, theta, 100.0, -20.0) + [SAFE[5]]


def test_solve_cyl_round_trip(ik, pose):
    servo = ik.solve_cyl(pose, 310.0, 5.0, 110.0, -10.0)
    r2, t2, h2 = ik.cylindrical(servo)
    assert abs(r2 - 310) < 1 and abs(t2 - 5) < 1 and abs(h2 - 110) < 1
    assert abs(ik.tool_pitch_deg(servo) + 10) < 1.1


def test_level_j3(ik, pose):
    p = list(pose)
    p[3] = ik.level_j3(pose)
    assert abs(ik.tool_pitch_deg(p)) < 0.1


def test_set_level_queues_level_pose_and_roll(ik, pose):
    svc = service(ik, pose)
    svc.set_level(True, 28.0)
    assert svc.level_enabled
    p = list(pose)
    for i, v in svc._pending.items():
        p[i] = v
    assert abs(ik.tool_pitch_deg(p)) < 1.1
    assert svc._pending[4] == 28.0


def test_set_level_at_unreachable_home_is_blocked(ik):
    svc = service(ik)
    with pytest.raises(ValueError, match="blocked"):
        svc.set_level(True)
    assert not svc.level_enabled and svc._pending == {}


def test_raw_j2_move_keeps_level(ik, pose):
    svc = service(ik, pose)
    svc.level_enabled = True
    svc.move({2: pose[2] + 5})
    p = list(pose)
    for i, v in svc._pending.items():
        p[i] = v
    assert abs(ik.tool_pitch_deg(p)) < 0.1


def test_unreachable_is_blocked(ik):
    svc = service(ik)
    with pytest.raises(ValueError, match="blocked"):
        svc.move_cyl(900.0, 0.0, 100.0, 0.0)
    assert svc._pending == {}


def test_floor_blocks_cyl_move(ik):
    svc = service(ik)
    r, theta, _ = ik.cylindrical(SAFE)
    with pytest.raises(ValueError, match="blocked"):
        svc.move_cyl(r, theta, -40.0, -30.0)
    assert svc._pending == {}


def test_raw_move_without_level(ik):
    svc = service(ik)
    svc.move({0: 100.0})
    assert svc._pending == {0: 100.0}
