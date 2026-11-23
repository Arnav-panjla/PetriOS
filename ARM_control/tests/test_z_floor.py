"""Z-floor guard: slider/home moves may not drive the TCP below the floor."""

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
        self.moved = None

    def move_all(self, pose):
        self.moved = list(pose)


def service(ik, pose=SAFE):
    svc = ArmService.__new__(ArmService)
    svc.arm = FakeArm(pose)
    svc.ik = ik
    svc.floor_enabled = True
    svc.floor_mm = 5.0
    svc.level_enabled = False
    svc.home_deg = list(SAFE)
    import threading

    svc._lock = threading.Lock()
    svc._pending = {}
    svc._wake = threading.Event()
    return svc


def test_safe_pose_is_above_floor(ik):
    assert ik.tcp_position(SAFE)[2] * 1000 > 5.0


def test_low_shoulder_is_blocked(ik):
    svc = service(ik)
    with pytest.raises(ValueError, match="blocked"):
        svc.move({1: 140.0})
    assert svc._pending == {}


def test_small_move_passes(ik):
    svc = service(ik)
    svc.move({0: 100.0})
    assert svc._pending == {0: 100.0}


def test_guard_off_allows_everything(ik):
    svc = service(ik)
    svc.floor_enabled = False
    svc.move({1: 140.0})
    assert svc._pending == {1: 140.0}


def test_pending_targets_are_combined(ik):
    svc = service(ik)
    svc._pending = {1: 140.0}  # queued earlier, not yet executed
    with pytest.raises(ValueError, match="blocked"):
        svc.move({0: 100.0})


def test_path_dip_between_safe_endpoints(ik):
    # J1 and J2 swing together: both endpoints are above the floor, the path between is not
    lo = list(SAFE)
    lo[1], lo[2] = 10.0, 40.0
    hi = list(SAFE)
    hi[1], hi[2] = 130.0, 10.0
    ends = [ik.tcp_position(p)[2] * 1000 for p in (lo, hi)]
    start_z, min_z, _ = ik.path_min_z(lo, hi)
    if min(ends) <= 5.0 or min_z >= 5.0:
        pytest.skip("geometry assumption changed")
    svc = service(ik, lo)
    with pytest.raises(ValueError, match="blocked"):
        svc.move({1: hi[1], 2: hi[2]})
