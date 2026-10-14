"""Command line for the six-servo arm."""

from __future__ import annotations

import argparse
import logging
import sys
from collections.abc import Callable
from pathlib import Path

from robot_arm.config import RobotConfig
from robot_arm.control.arm_controller import STARTUP_WARNING, ArmController
from robot_arm.hardware.driver import PCA9685Driver, PWMDriver
from robot_arm.utils.errors import JointLimitError, RobotArmError

logger = logging.getLogger(__name__)


def main(
    argv: list[str] | None = None,
    *,
    driver: PWMDriver | None = None,
    clock: Callable[[], float] | None = None,
    sleep: Callable[[float], None] | None = None,
    input_fn: Callable[[str], str] = input,
) -> int:
    """Run the arm CLI. A passed-in driver is used instead of the PCA9685."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s %(name)s: %(message)s",
    )
    parser = argparse.ArgumentParser(
        description=(
            "Command the six arm servos on PCA9685 channels 10-15. "
            "Angles are degrees across each joint's own pulse range. "
            "There is no position feedback."
        )
    )
    parser.add_argument("joint", nargs="?", type=int, help="joint 0-5")
    parser.add_argument("degrees", nargs="?", type=float, help="target angle, 0-180")
    parser.add_argument(
        "--yes",
        action="store_true",
        help="skip the startup question and snap to the stored home pulses",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="solve kinematics and print poses without writing servo pulses",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="directory that contains joints.yaml, hardware.yaml, and motion.yaml",
    )
    args = parser.parse_args(argv)
    if (args.joint is None) != (args.degrees is None):
        parser.error("pass both joint and degrees, or neither for the prompt")

    config = RobotConfig.load(args.config)
    if args.dry_run and driver is None:
        from robot_arm.hardware.driver import MockDriver

        driver = MockDriver()
        owns_driver = False
    owns_driver = driver is None
    if driver is None:
        driver = PCA9685Driver(config)
    arm_kwargs: dict[str, Callable] = {}
    if clock is not None:
        arm_kwargs["clock"] = clock
    if sleep is not None:
        arm_kwargs["sleep"] = sleep
    arm = ArmController(config, driver, **arm_kwargs)
    arm.dry_run = args.dry_run
    try:
        if not args.dry_run and not _confirm_startup(config, args.yes, input_fn):
            logger.info("startup declined; no PWM written")
            return 0
        if not args.dry_run:
            arm.startup(confirmed=True)
        if args.joint is None:
            return _repl(arm, input_fn)
        assert args.degrees is not None
        arm.move_joint(args.joint, args.degrees)
        return 0
    except KeyboardInterrupt:
        print(
            "\nstopped. The last pulses were sent; "
            "the arm may not have reached them."
        )
        return 130
    finally:
        if owns_driver:
            driver.close()


def _confirm_startup(
    config: RobotConfig,
    assume_yes: bool,
    input_fn: Callable[[str], str],
) -> bool:
    print(STARTUP_WARNING)
    if not config.require_confirmation or assume_yes:
        return True
    try:
        answer = input_fn("Snap to the stored home pose? [y/N] ")
    except EOFError:
        print()
        return False
    return answer.strip().lower() in {"y", "yes"}


def _repl(arm: ArmController, input_fn: Callable[[str], str]) -> int:
    print("type help for commands. q to quit.")
    while True:
        try:
            line = input_fn("arm> ")
        except EOFError:
            print()
            return 0
        except KeyboardInterrupt:
            print()
            return 0
        try:
            if not handle_line(arm, line):
                return 0
        except KeyboardInterrupt:
            print(
                "\nstopped. The last pulses were sent; "
                "the arm may not have reached them."
            )
        except (RobotArmError, ValueError) as exc:
            print(exc)


def handle_line(arm: ArmController, line: str) -> bool:
    """Apply one interactive command. Return False when the user quits."""
    parts = line.split()
    if not parts:
        return True
    command = parts[0].lower()
    if command in {"q", "quit", "exit"}:
        return False
    if command in {"help", "?"}:
        _print_help()
        return True
    if command == "status":
        print(arm.format_status())
        return True
    if command == "config":
        print(arm.format_config())
        return True
    if command == "home":
        if arm.dry_run:
            print("dry-run: home would move to the stored home pulses")
            return True
        arm.home()
        return True
    if command == "fk" or command == "pose":
        _print_pose(arm)
        return True
    if command == "joints":
        print(arm.format_status())
        return True
    if command == "ik":
        _print_ik(arm, parts[1:], move=False)
        return True
    if command == "goto":
        _print_ik(arm, parts[1:], move=not arm.dry_run)
        return True
    if command == "calibration":
        _calibration(arm, parts[1:])
        return True
    if command == "soften":
        _set_soften(arm, parts)
        return True
    if command == "move":
        _move_one(arm, parts[1:])
        return True
    if command == "moveall":
        _move_all(arm, parts[1:])
        return True
    if len(parts) == 2:
        _move_one(arm, parts)
        return True
    print(
        "usage: <joint> <degrees> | move | moveall | "
        "home | status | config | soften | q"
    )
    return True


def _print_help() -> None:
    print(
        "commands:\n"
        "  <joint 0-5> <degrees>     quintic move, other joints hold\n"
        "  move <joint> <degrees>\n"
        "  moveall d0 d1 d2 d3 d4 d5\n"
        "  home                      quintic return to the stored home pulses\n"
        "  fk, pose                  cylindrical pose of the commanded joints\n"
        "  joints                    commanded joint angles\n"
        "  ik r theta z pitch        solve only, r and z in mm, angles in deg\n"
        "  goto r theta z pitch      solve, check, then move\n"
        "  calibration show\n"
        "  calibration set servo_1.zero_offset 5\n"
        "  status                    commanded angles (not measured)\n"
        "  config                    calibration and speed limits\n"
        "  soften <ms>               ms of travel per degree (0 jumps)\n"
        "  q"
    )


def _print_pose(arm: ArmController) -> None:
    pose = arm.get_pose()
    x, y, z = pose.position_mm
    print(
        f"r {pose.r_mm:.2f} mm  theta {pose.theta_deg:.2f} deg  "
        f"z {pose.z_mm:.2f} mm  tool_pitch {pose.tool_pitch_deg:.2f} deg"
    )
    print(f"base xyz {x:.2f} {y:.2f} {z:.2f} mm")
    print("commanded pose only; servos have no position feedback")


def _print_ik(arm: ArmController, parts: list[str], *, move: bool) -> None:
    if len(parts) not in (3, 4):
        print("usage: ik r_mm theta_deg z_mm [tool_pitch_deg]")
        return
    try:
        radius = float(parts[0])
        theta = float(parts[1])
        height = float(parts[2])
        pitch = float(parts[3]) if len(parts) == 4 else 0.0
    except ValueError:
        print("usage: ik r_mm theta_deg z_mm [tool_pitch_deg]")
        return
    solution = arm.kinematics().ik(
        radius,
        theta,
        height,
        pitch,
        current_servo_deg=tuple(joint.commanded_deg for joint in arm._joints),
    )
    angles = " ".join(f"{angle:.2f}" for angle in solution.servo_deg)
    print(
        f"{solution.elbow} via {solution.source}: {angles} deg  "
        f"residual {solution.residual_mm:.3f} mm  "
        f"pitch {solution.residual_pitch_deg:.3f} deg"
    )
    if solution.warning:
        print(f"warning: {solution.warning}")
    if move:
        arm.move_all(solution.servo_deg)


def _calibration(arm: ArmController, parts: list[str]) -> None:
    if not parts or parts[0] == "show":
        print(arm.calibration_show())
        return
    if parts[0] == "set" and len(parts) == 3:
        arm.calibration_set(parts[1], parts[2])
        print(f"saved {parts[1]} = {parts[2]}")
        return
    print("usage: calibration show | calibration set servo_1.zero_offset 5")


def _set_soften(arm: ArmController, parts: list[str]) -> None:
    if len(parts) != 2:
        current = (
            "yaml limits" if arm.soften_ms is None else f"{arm.soften_ms:g} ms/deg"
        )
        print(f"soften is {current}. usage: soften <ms>")
        return
    try:
        value = float(parts[1])
    except ValueError:
        print("soften wants milliseconds per degree, for example: soften 40")
        return
    if value < 0:
        print("soften must be >= 0")
        return
    arm.soften_ms = value
    print(f"soften {arm.soften_ms:g} ms per degree")


def _move_one(arm: ArmController, parts: list[str]) -> None:
    if len(parts) != 2:
        print("usage: move <joint 0-5> <degrees>")
        return
    try:
        index = int(parts[0])
        degrees = float(parts[1])
    except ValueError:
        print("usage: move <joint 0-5> <degrees>")
        return
    try:
        arm.move_joint(index, degrees)
    except JointLimitError as exc:
        print(exc)


def _move_all(arm: ArmController, parts: list[str]) -> None:
    if len(parts) != 6:
        print("usage: moveall d0 d1 d2 d3 d4 d5")
        return
    try:
        angles = [float(part) for part in parts]
    except ValueError:
        print("usage: moveall d0 d1 d2 d3 d4 d5")
        return
    arm.move_all(angles)


if __name__ == "__main__":
    sys.exit(main())
