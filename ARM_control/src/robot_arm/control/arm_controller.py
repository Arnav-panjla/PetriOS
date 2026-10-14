"""Six-joint open-loop arm. Startup snap and later moves are different."""

from __future__ import annotations

import logging
import time
from collections.abc import Callable, Sequence

from robot_arm.config import RobotConfig
from robot_arm.control.joint_controller import JointController
from robot_arm.control.safety import SafetyManager
from robot_arm.hardware.driver import PWMDriver
from robot_arm.hardware.servo import Servo
from robot_arm.motion.planner import TrajectoryPlanner
from robot_arm.motion.trajectory import run_trajectory
from robot_arm.utils.errors import NotReadyError

logger = logging.getLogger(__name__)

STARTUP_WARNING = (
    "The physical arm position is unknown. "
    "The next command snaps all six servos to the stored home pulses "
    "and may move the arm abruptly. "
    "This is not a measured homing procedure."
)


class ArmController:
    """Coordinates the six joints. Home pulses stay in the YAML files."""

    def __init__(
        self,
        config: RobotConfig,
        driver: PWMDriver,
        *,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        planner: TrajectoryPlanner | None = None,
    ) -> None:
        self.config = config
        self._driver = driver
        self._clock = clock
        self._sleep = sleep
        self._planner = planner or TrajectoryPlanner()
        self._safety = SafetyManager(config.require_confirmation)
        self._joints = tuple(
            JointController(Servo(joint), driver, self._safety)
            for joint in config.joints
        )
        # None means "use the velocity and acceleration limits in motion.yaml".
        self.soften_ms: float | None = None

    @property
    def ready(self) -> bool:
        return self._safety.pwm_allowed

    def startup(self, *, confirmed: bool) -> None:
        """Snap to the stored home pulses after an explicit confirmation.

        Commanded angles are already the home angles. This writes them.
        It does not interpolate, because the physical pose is unknown.
        """
        if self.config.require_confirmation and not confirmed:
            raise NotReadyError(STARTUP_WARNING)
        logger.warning(STARTUP_WARNING)
        self._safety.authorize_pwm()
        home = tuple(joint.servo.calibration.home_deg for joint in self._joints)
        logger.info("startup snap to stored home pulses")
        self._write_all(home)

    def home(self) -> None:
        """Quintic return to the stored home angles from the last commands."""
        targets = [joint.servo.calibration.home_deg for joint in self._joints]
        self.move_all(targets)

    def move_joint(self, index: int, degrees: float) -> None:
        self._safety.check_index(index, len(self._joints))
        targets = [joint.commanded_deg for joint in self._joints]
        targets[index] = degrees
        self.move_all(targets)

    def move_all(self, degrees: Sequence[float]) -> None:
        self._safety.ensure_pwm_allowed()
        if len(degrees) != len(self._joints):
            raise ValueError(
                f"expected {len(self._joints)} angles, got {len(degrees)}"
            )
        targets = [self._safety.clamp_angle(angle) for angle in degrees]
        starts = tuple(joint.commanded_deg for joint in self._joints)
        joints = tuple(joint.servo.calibration for joint in self._joints)
        trajectory = self._planner.plan(
            starts,
            targets,
            joints,
            soften_ms=self.soften_ms,
        )
        logger.info("trajectory duration %.3f s", trajectory.duration_s)
        period = 1.0 / self.config.control_hz
        run_trajectory(
            trajectory,
            self._write_all,
            period_s=period,
            clock=self._clock,
            sleep=self._sleep,
        )

    def format_status(self) -> str:
        if self.soften_ms is None:
            timing = "timing: velocity and acceleration limits"
        else:
            timing = f"timing: soften {self.soften_ms:g} ms per degree"
        lines = [
            "commanded angles only; these servos have no position feedback",
            timing,
        ]
        for joint in self._joints:
            calibration = joint.servo.calibration
            lines.append(
                f"joint {calibration.index}  ch {calibration.channel:2d}  "
                f"{joint.commanded_deg:7.2f} deg "
                f"({joint.servo.commanded_pulse_us():4d} us)  "
                f"home {calibration.home_us:4d} us  "
                f"limits {calibration.min_us}-{calibration.max_us} us"
            )
        return "\n".join(lines)

    def format_config(self) -> str:
        hardware = self.config.hardware
        lines = [
            f"i2c bus {hardware.i2c_bus}  address 0x{hardware.address:02X}  "
            f"pwm {hardware.pwm_frequency_hz:g} Hz  "
            f"control {self.config.control_hz:g} Hz",
            f"startup confirmation required: {hardware.require_confirmation}",
            "home pulses are fixed in configs/joints.yaml",
        ]
        for joint in self.config.joints:
            lines.append(
                f"joint {joint.index}  ch {joint.channel:2d}  "
                f"home {joint.home_us} us ({joint.home_deg:.2f} deg)  "
                f"limits {joint.min_us}-{joint.max_us} us  "
                f"vmax {joint.max_velocity_deg_s:g} deg/s  "
                f"amax {joint.max_acceleration_deg_s2:g} deg/s^2"
            )
        return "\n".join(lines)

    def _write_all(self, degrees: Sequence[float]) -> None:
        for joint, angle in zip(self._joints, degrees, strict=True):
            joint.command(angle)
