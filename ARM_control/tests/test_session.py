"""Session recorder file format and playback through the GUI service."""

import time

import pytest

from robot_arm.config import RobotConfig
from robot_arm.control.arm_controller import ArmController
from robot_arm.hardware.driver import NoOpDriver
from robot_arm.interfaces.web_gui import ArmService, IKBridge
from robot_arm.motion.session import SessionRecorder
from robot_arm.utils.errors import RobotArmError


def test_recorder_writes_and_reads_back(tmp_path) -> None:
    rec = SessionRecorder(tmp_path)
    with pytest.raises(ValueError):
        rec.record([0] * 6)  # no session yet
    path = rec.start()
    rec.record([1, 2, 3, 4, 5, 6])
    rec.record([10.5, 20, 30, 40, 50, 60])
    assert rec.points[1]["n"] == 2
    rec.end()
    assert rec.files() == [path.name]
    loaded = rec.load(path.name)
    assert [p["angles"] for p in loaded] == [[1, 2, 3, 4, 5, 6], [10.5, 20, 30, 40, 50, 60]]
    assert "n,time,j0,j1,j2,j3,j4,j5" in path.read_text()


def test_recorder_rejects_bad_input(tmp_path) -> None:
    rec = SessionRecorder(tmp_path)
    rec.start()
    with pytest.raises(ValueError):
        rec.record([1, 2, 3])  # wrong joint count
    with pytest.raises(ValueError):
        rec.load("../etc/passwd")  # only names from files() are accepted


def make_service(tmp_path) -> ArmService:
    arm = ArmController(RobotConfig.load(), NoOpDriver(), sleep=lambda _s: None)
    arm.startup_simulation()
    arm.soften_ms = 0
    svc = ArmService(arm, IKBridge(), saved_home=False)
    svc.floor_enabled = False  # keep this test independent of the IK solver
    svc.session = SessionRecorder(tmp_path)
    return svc


def wait_idle(svc: ArmService) -> None:
    deadline = time.monotonic() + 10
    while svc._playing:
        assert time.monotonic() < deadline, "playback did not finish"
        time.sleep(0.02)


def test_record_then_play_returns_to_recorded_poses(tmp_path) -> None:
    svc = make_service(tmp_path)
    with pytest.raises(RobotArmError):
        svc.session_record()  # needs a session

    svc.session_start()
    base = svc.commanded()
    first = list(base)
    first[0] += 10
    svc.arm.move_all(first)
    svc.session_record()
    second = list(first)
    second[1] += 5
    svc.arm.move_all(second)
    svc.session_record()
    svc.session_end()

    svc.arm.move_all(base)
    svc.session_play()
    wait_idle(svc)

    assert svc.commanded() == pytest.approx(second, abs=0.01)  # ended on the last point
    assert "done" in svc.session_status()["message"]
    assert len(svc.session_status()["points"]) == 2


def test_manual_moves_are_locked_while_playing(tmp_path) -> None:
    svc = make_service(tmp_path)
    svc._playing = True
    with pytest.raises(RobotArmError):
        svc.move({0: 90.0})
    with pytest.raises(RobotArmError):
        svc.home()


def test_play_rejects_points_outside_joint_limits(tmp_path) -> None:
    svc = make_service(tmp_path)
    svc.session.start()
    svc.session.record([90, 90, 90, 90, 90, 500])  # gripper far outside its range
    svc.session.end()
    with pytest.raises(ValueError):
        svc.session_play()
    assert not svc._playing
