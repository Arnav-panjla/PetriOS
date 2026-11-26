"""Hold: freeze the arm mid-move for a while, then let the move carry on."""

import threading
import time

import pytest

from robot_arm.config import RobotConfig
from robot_arm.control.arm_controller import ArmController
from robot_arm.hardware.driver import NoOpDriver
from robot_arm.interfaces.web_gui import ArmService, IKBridge
from robot_arm.motion.pause import PausableClock
from robot_arm.utils.errors import RobotArmError


def test_clock_stops_while_paused() -> None:
    now = [100.0]
    clock = PausableClock(base=lambda: now[0])
    now[0] = 101.0
    start = clock()
    clock.pause()
    now[0] += 5.0
    assert clock() == start  # frozen
    clock.resume()
    now[0] += 1.0
    assert clock() == pytest.approx(start + 1.0)  # the 5 s pause is not counted
    clock.resume()  # harmless when not paused
    assert not clock.paused


def make_arm():
    clock = PausableClock()
    arm = ArmController(RobotConfig.load(), NoOpDriver(), clock=clock)
    arm.startup_simulation()
    arm.soften_ms = 20  # about 20 ms per degree, so a 50 degree move takes about 1 s
    return arm, clock


def test_move_holds_position_then_finishes() -> None:
    arm, clock = make_arm()
    start = [j.commanded_deg for j in arm._joints]
    target = list(start)
    target[0] = start[0] + 50

    worker = threading.Thread(target=arm.move_all, args=(target,))
    worker.start()
    time.sleep(0.3)
    clock.pause()
    time.sleep(0.1)  # let the loop settle on the frozen pose
    held = [j.commanded_deg for j in arm._joints]
    assert start[0] < held[0] < target[0]  # stopped part-way
    time.sleep(0.6)
    assert [j.commanded_deg for j in arm._joints] == held  # did not creep while held
    assert worker.is_alive()

    clock.resume()
    worker.join(timeout=5)
    assert not worker.is_alive()
    assert [j.commanded_deg for j in arm._joints][0] == pytest.approx(target[0], abs=0.01)


def test_service_hold_and_resume() -> None:
    arm, clock = make_arm()
    svc = ArmService(arm, IKBridge(), saved_home=False)
    with pytest.raises(RobotArmError):
        svc.hold(1)  # no clock wired yet
    svc.pause_clock = clock
    with pytest.raises(ValueError):
        svc.hold(0)
    with pytest.raises(ValueError):
        svc.hold(601)

    svc.hold(30)
    assert clock.paused and svc.hold_left() > 25
    svc.resume()
    assert not clock.paused and svc.hold_left() == 0

    svc.hold(0.2)  # ends by itself
    time.sleep(0.5)
    assert not clock.paused
