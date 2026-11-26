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
    assert [p["hold_s"] for p in loaded] == [0, 0]
    assert [p["event"] for p in loaded] == [None, None]
    assert "n,time,j0,j1,j2,j3,j4,j5" in path.read_text()


def test_hold_is_saved_on_the_last_point_and_survives_reload(tmp_path) -> None:
    rec = SessionRecorder(tmp_path)
    path = rec.start()
    with pytest.raises(ValueError):
        rec.set_hold(5)  # nothing recorded yet
    rec.record([1, 2, 3, 4, 5, 6])
    rec.record([7, 8, 9, 10, 11, 12])
    rec.set_hold(5)  # after the 2nd pose
    rec.record([13, 14, 15, 16, 17, 18])
    rec.end()
    assert [p["hold_s"] for p in rec.load(path.name)] == [0, 5, 0]
    for bad in (0, -1, 601):
        rec2 = SessionRecorder(tmp_path)
        rec2.start()
        rec2.record([0] * 6)
        with pytest.raises(ValueError):
            rec2.set_hold(bad)


def test_old_files_without_hold_column_still_load(tmp_path) -> None:
    (tmp_path / "session_old.csv").write_text("n,stage,j0,j1,j2,j3,j4,j5\n1,a,1,2,3,4,5,6\n2,b,1,2,3,4,5,6\n")
    pts = SessionRecorder(tmp_path).load("session_old.csv")
    assert [p["hold_s"] for p in pts] == [0, 0]
    (tmp_path / "session_manual.csv").write_text("n,stage,j0,j1,j2,j3,j4,j5,hold_s\n1,a,1,2,3,4,5,6,2.5\n")
    assert SessionRecorder(tmp_path).load("session_manual.csv")[0]["hold_s"] == 2.5


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


def test_play_waits_at_the_recorded_hold(tmp_path) -> None:
    svc = make_service(tmp_path)
    svc.session_start()
    pose1 = list(svc.commanded())
    pose1[0] += 10
    svc.arm.move_all(pose1)
    svc.session_record()
    svc.hold_or_record(0.6)  # recording: this saves a hold, it does not freeze the arm
    assert not (svc.pause_clock and svc.pause_clock.paused)
    pose2 = list(pose1)
    pose2[1] += 5
    svc.arm.move_all(pose2)
    svc.session_record()
    svc.session_end()
    assert [p["hold_s"] for p in svc.session_status()["points"]] == [0.6, 0]

    svc.arm.move_all(list(svc.home_deg))
    started = time.monotonic()
    svc.session_play()
    saw_hold = False
    while svc._playing:
        saw_hold = saw_hold or svc.hold_left() > 0
        time.sleep(0.02)
    assert time.monotonic() - started >= 0.55
    assert saw_hold  # the GUI can show the countdown
    assert svc.commanded() == pytest.approx(pose2, abs=0.01)


def test_stop_ends_a_hold_during_playback(tmp_path) -> None:
    svc = make_service(tmp_path)
    svc.session.start()
    svc.session.record(list(svc.commanded()))
    svc.session.set_hold(30)
    svc.session.end()
    svc.session_play()
    time.sleep(0.3)
    assert svc._playing and svc.hold_left() > 0
    svc.session_stop()
    wait_idle(svc)
    assert "Stopped" in svc.session_status()["message"] or "done" in svc.session_status()["message"]


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


def test_playback_holds_at_a_pose_then_continues(tmp_path) -> None:
    svc = make_service(tmp_path)
    home = svc.commanded()
    first = list(home); first[0] += 1
    second = list(home); second[0] += 2
    path = tmp_path / "session_qr_hold.csv"
    path.write_text(
        "n,stage,j0,j1,j2,j3,j4,j5,hold_s\n"
        + "1,open," + ",".join(map(str, first)) + ",0.3\n"
        + "2,enter," + ",".join(map(str, second)) + ",0\n",
        encoding="utf-8",
    )
    svc.session_play(path.name)
    deadline = time.monotonic() + 3
    while "Holding at point 1" not in svc.session_status()["message"]:
        assert time.monotonic() < deadline
        time.sleep(0.01)
    assert svc.commanded() == pytest.approx(first)
    wait_idle(svc)
    assert svc.commanded() == pytest.approx(second)


def test_playback_stop_interrupts_a_hold(tmp_path) -> None:
    svc = make_service(tmp_path)
    home = svc.commanded()
    next_pose = list(home); next_pose[0] += 2
    path = tmp_path / "session_stop_hold.csv"
    path.write_text(
        "n,stage,j0,j1,j2,j3,j4,j5,hold_s\n"
        + "1,open," + ",".join(map(str, home)) + ",6\n"
        + "2,enter," + ",".join(map(str, next_pose)) + ",0\n",
        encoding="utf-8",
    )
    svc.session_play(path.name)
    deadline = time.monotonic() + 3
    while "Holding at point 1" not in svc.session_status()["message"]:
        assert time.monotonic() < deadline
        time.sleep(0.01)
    svc.session_stop()
    wait_idle(svc)
    assert svc.commanded() == pytest.approx(home)
    assert "Stopped" in svc.session_status()["message"]


def test_playback_waits_for_gazebo_dish_events(tmp_path) -> None:
    svc = make_service(tmp_path)
    svc.simulation_only = True
    svc.petri_bridge_heartbeat()
    home = svc.commanded()
    first = list(home); first[0] += 1
    second = list(home); second[0] += 2
    third = list(home); third[0] += 3
    path = tmp_path / "session_dish_events.csv"
    path.write_text(
        "n,stage,j0,j1,j2,j3,j4,j5,hold_s,event\n"
        + "1,grip," + ",".join(map(str, first)) + ",0,attach\n"
        + "2,carry," + ",".join(map(str, second)) + ",0,\n"
        + "3,release," + ",".join(map(str, third)) + ",0,detach\n",
        encoding="utf-8",
    )
    svc.session_play(path.name)
    for event, pose in (("attach", first), ("detach", third)):
        deadline = time.monotonic() + 3
        while svc.petri_status()["event_kind"] != event:
            assert time.monotonic() < deadline
            time.sleep(0.01)
        status = svc.petri_status()
        assert status["active"]
        assert svc.commanded() == pytest.approx(pose)
        svc.petri_event_ack(status["event_seq"])
    wait_idle(svc)
    assert svc.commanded() == pytest.approx(third)
    assert "done" in svc.session_status()["message"]


def test_simulated_dish_event_requires_gazebo_bridge(tmp_path) -> None:
    svc = make_service(tmp_path)
    svc.simulation_only = True
    path = tmp_path / "session_dish_event.csv"
    path.write_text(
        "n,stage,j0,j1,j2,j3,j4,j5,hold_s,event\n"
        + "1,grip," + ",".join(map(str, svc.commanded())) + ",0,attach\n",
        encoding="utf-8",
    )
    with pytest.raises(RobotArmError, match="bridge is not connected"):
        svc.session_play(path.name)
    assert not svc._playing
