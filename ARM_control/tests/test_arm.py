"""Mock-driver tests for calibration, quintic motion, and startup.

Run with pytest from ARM_control/robotic-arm. This file is the test runner's
entry for the cases listed in the refactor plan. It does not read production
logs. Expected home pulses are the integers in configs/joints.yaml.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from robot_arm.config import RobotConfig
from robot_arm.control.arm_controller import ArmController
from robot_arm.hardware.driver import MockDriver
from robot_arm.hardware.servo import deg_to_us, us_to_deg
from robot_arm.interfaces.cli import handle_line, main
from robot_arm.motion.interpolators import (
    minimum_duration,
    quintic,
    quintic_acceleration,
    quintic_velocity,
)
from robot_arm.motion.trajectory import Trajectory
from robot_arm.utils.errors import ConfigError, JointLimitError, NotReadyError

ROOT = Path(__file__).resolve().parents[1]
CONFIGS = ROOT / "configs"

HOME = (
    (0, 10, 500, 2900, 1700),
    (1, 11, 500, 2700, 900),
    (2, 12, 500, 2900, 1700),
    (3, 13, 500, 2900, 2000),
    (4, 14, 500, 2900, 2050),
    (5, 15, 1100, 1600, 1239),
)


class ManualClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        if seconds > 0:
            self.now += seconds


class InterruptClock(ManualClock):
    def __init__(self, fail_on_sleep: int) -> None:
        super().__init__()
        self.fail_on_sleep = fail_on_sleep
        self.calls = 0

    def sleep(self, seconds: float) -> None:
        self.calls += 1
        if self.calls >= self.fail_on_sleep:
            raise KeyboardInterrupt
        super().sleep(seconds)


def make_arm(clock: ManualClock | None = None) -> tuple[ArmController, MockDriver]:
    config = RobotConfig.load(CONFIGS)
    ticker = clock or ManualClock()
    driver = MockDriver(clock=ticker.monotonic)
    arm = ArmController(
        config,
        driver,
        clock=ticker.monotonic,
        sleep=ticker.sleep,
    )
    return arm, driver


def frames(driver: MockDriver) -> list[list]:
    grouped: list[list] = []
    for write in driver.writes:
        if not grouped or grouped[-1][0].time_s != write.time_s:
            grouped.append([write])
        else:
            grouped[-1].append(write)
    return grouped


def test_home_pulses_round_trip() -> None:
    config = RobotConfig.load(CONFIGS)
    assert config.hardware.address == 0x40
    assert config.hardware.i2c_bus == 1
    assert config.hardware.pwm_frequency_hz == 50
    assert config.control_hz == 50
    assert config.require_confirmation is True
    assert len(config.joints) == 6
    for joint, expected in zip(config.joints, HOME, strict=True):
        index, channel, min_us, max_us, home_us = expected
        assert (
            joint.index,
            joint.channel,
            joint.min_us,
            joint.max_us,
            joint.home_us,
        ) == (index, channel, min_us, max_us, home_us)
        assert int(round(deg_to_us(min_us, max_us, joint.home_deg))) == home_us


def test_missing_config(tmp_path: Path) -> None:
    with pytest.raises(ConfigError):
        RobotConfig.load(tmp_path)


def test_quintic_endpoints_and_zero_end_derivatives() -> None:
    assert quintic(0.0) == 0.0
    assert quintic(1.0) == 1.0
    assert quintic(-1.0) == 0.0
    assert quintic(2.0) == 1.0
    assert quintic_velocity(0.0) == 0.0
    assert quintic_velocity(1.0) == 0.0
    assert quintic_acceleration(0.0) == 0.0
    assert quintic_acceleration(1.0) == 0.0
    assert quintic_velocity(0.5) == pytest.approx(1.875)

    duration = minimum_duration(30.0, 60.0, 200.0)
    assert duration == pytest.approx(1.875 * 30.0 / 60.0)


def test_shared_tau_on_two_joints() -> None:
    trajectory = Trajectory(
        starts=(10.0, 0.0, 5.0),
        targets=(40.0, 20.0, 5.0),
        duration_s=1.0,
    )
    sample = trajectory.sample(0.3)
    progress_0 = (sample[0] - 10.0) / 30.0
    progress_1 = (sample[1] - 0.0) / 20.0
    assert progress_0 == pytest.approx(progress_1)
    assert sample[2] == pytest.approx(5.0)
    assert trajectory.sample(1.0) == pytest.approx([40.0, 20.0, 5.0])


def test_no_pwm_before_startup_confirmation() -> None:
    arm, driver = make_arm()
    with pytest.raises(NotReadyError):
        arm.startup(confirmed=False)
    with pytest.raises(NotReadyError):
        arm.move_joint(0, 120)
    assert driver.writes == []


def test_startup_snaps_to_home_pulses() -> None:
    arm, driver = make_arm()
    arm.startup(confirmed=True)
    assert len(driver.writes) == 6
    for joint, write in zip(arm.config.joints, driver.writes, strict=True):
        assert write.channel == joint.channel
        assert write.pulse_us == pytest.approx(joint.home_us)


def test_synchronized_two_joint_move_ends_on_targets() -> None:
    arm, driver = make_arm()
    arm.startup(confirmed=True)
    driver.writes.clear()
    targets = [joint.home_deg for joint in arm.config.joints]
    targets[0] = 120.0
    targets[1] = 50.0
    arm.move_all(targets)

    grouped = frames(driver)
    assert len(grouped) > 2
    counts = {joint.channel: 0 for joint in arm.config.joints}
    for frame in grouped:
        assert len(frame) == 6
        assert len({write.time_s for write in frame}) == 1
        for write in frame:
            counts[write.channel] += 1
    assert len(set(counts.values())) == 1

    last = {write.channel: write.pulse_us for write in grouped[-1]}
    joint0, joint1 = arm.config.joints[0], arm.config.joints[1]
    assert last[joint0.channel] == pytest.approx(
        deg_to_us(joint0.min_us, joint0.max_us, 120.0)
    )
    assert last[joint1.channel] == pytest.approx(
        deg_to_us(joint1.min_us, joint1.max_us, 50.0)
    )
    assert arm._joints[0].commanded_deg == pytest.approx(120.0)
    assert arm._joints[1].commanded_deg == pytest.approx(50.0)

    mid = grouped[len(grouped) // 2]
    mid_pulse = {write.channel: write.pulse_us for write in mid}
    start0 = joint0.home_deg
    start1 = joint1.home_deg
    progress_0 = (
        us_to_deg(joint0.min_us, joint0.max_us, mid_pulse[joint0.channel]) - start0
    ) / (120.0 - start0)
    progress_1 = (
        us_to_deg(joint1.min_us, joint1.max_us, mid_pulse[joint1.channel]) - start1
    ) / (50.0 - start1)
    assert progress_0 == pytest.approx(progress_1)
    assert 0.0 < progress_0 < 1.0


def test_out_of_range_angle_never_passes_the_pulse_limit() -> None:
    arm, driver = make_arm()
    arm.startup(confirmed=True)
    driver.writes.clear()
    arm.soften_ms = 0
    arm.move_joint(0, 999)
    joint = arm.config.joints[0]
    assert driver.writes
    for write in driver.writes:
        if write.channel == joint.channel:
            assert joint.min_us - 1e-6 <= write.pulse_us <= joint.max_us + 1e-6
    last = [write for write in driver.writes if write.channel == joint.channel][-1]
    assert last.pulse_us == pytest.approx(joint.max_us)
    assert arm._joints[0].commanded_deg == pytest.approx(180.0)


def test_bad_joint_index_writes_nothing_more() -> None:
    arm, driver = make_arm()
    arm.startup(confirmed=True)
    written = len(driver.writes)
    with pytest.raises(JointLimitError):
        arm.move_joint(9, 10)
    assert len(driver.writes) == written


def test_home_does_not_rewrite_yaml_and_returns_to_home() -> None:
    before = (CONFIGS / "joints.yaml").read_bytes()
    arm, driver = make_arm()
    arm.startup(confirmed=True)
    arm.soften_ms = 0
    arm.move_joint(0, 120)
    arm.soften_ms = None
    driver.writes.clear()
    arm.home()
    assert (CONFIGS / "joints.yaml").read_bytes() == before
    last = {write.channel: write.pulse_us for write in frames(driver)[-1]}
    for joint in arm.config.joints:
        assert last[joint.channel] == pytest.approx(float(joint.home_us))
        assert arm._joints[joint.index].servo.commanded_pulse_us() == joint.home_us


def test_interrupt_keeps_last_command_not_the_target() -> None:
    clock = InterruptClock(fail_on_sleep=3)
    arm, driver = make_arm(clock)
    arm.startup(confirmed=True)
    driver.writes.clear()
    with pytest.raises(KeyboardInterrupt):
        arm.move_joint(0, 120)
    assert arm._joints[0].commanded_deg != pytest.approx(120.0)
    last_frame = frames(driver)[-1]
    by_channel = {write.channel: write.pulse_us for write in last_frame}
    joint = arm.config.joints[0]
    assert by_channel[joint.channel] == pytest.approx(
        deg_to_us(joint.min_us, joint.max_us, arm._joints[0].commanded_deg)
    )
    assert not math.isclose(arm._joints[0].commanded_deg, 120.0)


def test_cli_decline_writes_nothing() -> None:
    driver = MockDriver()
    code = main([], driver=driver, input_fn=lambda _prompt: "n")
    assert code == 0
    assert driver.writes == []


def test_cli_commands_and_one_shot() -> None:
    arm, driver = make_arm()
    arm.startup(confirmed=True)
    assert handle_line(arm, "q") is False
    handle_line(arm, "soften 0")
    assert arm.soften_ms == 0
    driver.writes.clear()
    assert handle_line(arm, "move 0 120") is True
    assert arm._joints[0].commanded_deg == pytest.approx(120.0)
    assert handle_line(arm, "0 90") is True
    assert arm._joints[0].servo.commanded_pulse_us() == 1700

    _arm, fresh_driver = make_arm()
    code = main(
        ["--yes", "0", "90"],
        driver=fresh_driver,
        clock=_arm._clock,
        sleep=_arm._sleep,
    )
    assert code == 0
    channel_10 = [write for write in fresh_driver.writes if write.channel == 10]
    assert channel_10[-1].pulse_us == pytest.approx(1700)
