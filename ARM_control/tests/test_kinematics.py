"""Hardware-free checks for URDF forward kinematics and cylindrical IK.

pytest collects this module. It reads configs/geometry.yaml and models/robot.urdf.
The calibration save test writes only a temporary copy.
"""

from __future__ import annotations

import math
import shutil
from pathlib import Path

import pytest
import yaml

from robot_arm.calibration import CalibrationStore
from robot_arm.calibration.joint_calibration import ServoMap
from robot_arm.config import RobotConfig
from robot_arm.control.arm_controller import ArmController
from robot_arm.hardware.driver import MockDriver
from robot_arm.kinematics.cylindrical import CylindricalKinematics
from robot_arm.kinematics.geometry import load_geometry
from robot_arm.kinematics.inverse import solve_elbow
from robot_arm.kinematics.transforms import deg, rad
from robot_arm.utils.errors import UnreachableError

CONFIGS = Path(__file__).resolve().parents[1] / "configs"
HOME_PULSES = (1700, 900, 1700, 2000, 2050, 1239)


class ManualClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        if seconds > 0:
            self.now += seconds


def _model(zero_offset: float = 0.0) -> CylindricalKinematics:
    store = CalibrationStore.load(CONFIGS)
    for servo in store.joints.servos:
        servo.zero_offset_deg = zero_offset
    return CylindricalKinematics(
        load_geometry(CONFIGS),
        store.joints,
        store.coordinates,
    )


def _servos(urdf_rad: tuple[float, ...], offset: float = 90.0) -> tuple[float, ...]:
    return tuple(offset + deg(angle) for angle in urdf_rad) + (offset,)


def test_fk_matches_urdf_chain_at_zero() -> None:
    model = _model(90.0)
    pose = model.fk(_servos((0, 0, 0, 0, 0)))
    geometry = model.geometry
    expected = [0.0, 0.0, 0.0]
    for joint in geometry.joints:
        expected = [left + right for left, right in zip(expected, joint.xyz_m, strict=True)]
    tool = geometry.tool_in_link_5_m
    expected = [left + right for left, right in zip(expected, tool, strict=True)]
    assert pose.position_m == pytest.approx(expected)
    assert pose.tool_pitch_deg == pytest.approx(0.0, abs=1e-6)


def test_ik_fk_round_trip() -> None:
    model = _model(90.0)
    original = (0.4, -0.3, 0.5, -0.2, 0.1)
    servos = _servos(original)
    pose = model.fk(servos)
    solution = model.ik(
        pose.r_mm,
        pose.theta_deg,
        pose.z_mm,
        pose.tool_pitch_deg,
        current_servo_deg=servos,  # type: ignore[arg-type]
    )
    assert solution.residual_mm < 0.05
    assert abs(solution.residual_pitch_deg) < 0.05
    again = model.fk(solution.servo_deg)
    assert again.r_mm == pytest.approx(pose.r_mm, abs=0.05)
    assert again.z_mm == pytest.approx(pose.z_mm, abs=0.05)
    assert again.theta_deg == pytest.approx(pose.theta_deg, abs=0.05)
    assert again.tool_pitch_deg == pytest.approx(pose.tool_pitch_deg, abs=0.05)


def test_both_elbow_branches_exist() -> None:
    geometry = load_geometry(CONFIGS)
    branches = solve_elbow((0.05, 0.12), geometry.upper_arm_m, geometry.forearm_m)
    assert len(branches) == 2
    assert branches[0][1] != pytest.approx(branches[1][1])


def test_ik_can_select_either_elbow() -> None:
    model = _model(0.0)
    model.joints.servos[1].zero_offset_deg = 40.0
    branches = solve_elbow(
        (0.16 * math.cos(2.0), 0.16 * math.sin(2.0)),
        model.geometry.upper_arm_m,
        model.geometry.forearm_m,
    )
    q2, q3 = branches[1]
    q4 = math.atan2(math.sin(0.2 - q2 - q3), math.cos(0.2 - q2 - q3))
    servos = model.joints.to_servo((0.3, q2, q3, q4, 0.0), (0, 0, 0, 0, 0, 50))
    pose = model.fk(servos)
    down = model.ik(
        pose.r_mm,
        pose.theta_deg,
        pose.z_mm,
        pose.tool_pitch_deg,
        current_servo_deg=servos,
        elbow="down",
    )
    up = model.ik(
        pose.r_mm,
        pose.theta_deg,
        pose.z_mm,
        pose.tool_pitch_deg,
        current_servo_deg=servos,
        elbow="up",
    )
    assert down.urdf_rad[2] != pytest.approx(up.urdf_rad[2])
    assert down.residual_mm < 0.05
    assert up.residual_mm < 0.05


def test_unreachable_target_is_rejected() -> None:
    model = _model(90.0)
    current = _servos((0, 0, 0, 0, 0))
    with pytest.raises(UnreachableError):
        model.ik(5000, 0, 0, 0, current_servo_deg=current)  # type: ignore[arg-type]


def test_direction_and_zero_offset() -> None:
    servo = ServoMap(1, "shoulder", "joint_2", direction=-1, zero_offset_deg=90.0)
    assert servo.to_servo(rad(10)) == pytest.approx(80.0)
    assert deg(servo.to_urdf(80.0)) == pytest.approx(10.0)
    servo.direction = 1
    assert servo.to_servo(rad(10)) == pytest.approx(100.0)


def test_cylindrical_frame_at_theta_zero() -> None:
    model = _model(90.0)
    pose = model.fk(_servos((0, 0, 0, 0, 0)))
    assert pose.theta_deg == pytest.approx(0.0, abs=1e-6)
    assert pose.r_mm > 0
    origin_y = model.geometry.joints[0].xyz_m[1]
    assert pose.position_m[1] == pytest.approx(origin_y - pose.r_mm / 1000.0)


def test_home_pulses_stay_in_the_config_file() -> None:
    loaded = yaml.safe_load((CONFIGS / "joints.yaml").read_text(encoding="utf-8"))
    pulses = tuple(row["home_us"] for row in loaded["joints"])
    assert pulses == HOME_PULSES


def test_goto_uses_one_synchronized_trajectory() -> None:
    clock = ManualClock()
    config = RobotConfig.load(CONFIGS)
    driver = MockDriver(clock=clock.monotonic)
    arm = ArmController(config, driver, clock=clock.monotonic, sleep=clock.sleep)
    arm.startup(confirmed=True)
    for servo in arm.kinematics().joints.servos:
        servo.zero_offset_deg = 90.0
    driver.writes.clear()
    pose = arm.get_pose()
    arm.move_to_cylindrical(
        pose.r_mm,
        pose.theta_deg,
        pose.z_mm,
        pose.tool_pitch_deg,
        duration=0.2,
    )
    times = {write.time_s for write in driver.writes}
    assert len(times) > 2
    counts = {channel: 0 for channel in range(10, 16)}
    for write in driver.writes:
        counts[write.channel] += 1
    assert len(set(counts.values())) == 1
    assert math.isclose(max(times) - min(times), 0.2, abs_tol=1e-9)


def test_dry_run_does_not_write_pwm() -> None:
    config = RobotConfig.load(CONFIGS)
    driver = MockDriver()
    arm = ArmController(config, driver)
    arm.dry_run = True
    for servo in arm.kinematics().joints.servos:
        servo.zero_offset_deg = 90.0
    pose = arm.get_pose()
    arm.move_to_cylindrical(pose.r_mm, 10, pose.z_mm, pose.tool_pitch_deg)
    assert driver.writes == []


def test_calibration_set_is_persistent(tmp_path: Path) -> None:
    copied = tmp_path / "calibration.yaml"
    shutil.copy(CONFIGS / "calibration.yaml", copied)
    store = CalibrationStore.load(CONFIGS)
    store.joints.path = copied
    store.coordinates.path = copied
    before = (CONFIGS / "calibration.yaml").read_text(encoding="utf-8")
    store.set_value("servo_1.zero_offset", "5")
    assert store.joints.servos[1].zero_offset_deg == 5
    saved = yaml.safe_load(copied.read_text(encoding="utf-8"))
    assert saved["servos"][1]["zero_offset_deg"] == 5
    assert (CONFIGS / "calibration.yaml").read_text(encoding="utf-8") == before
