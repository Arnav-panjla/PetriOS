"""
Six-servo arm on a PCA9685, Raspberry Pi version of esp32_slider.ino.

The sketch's pulse widths are microseconds. This CLI speaks degrees.
For each motor, 0 deg is that motor's min pulse and 180 deg is its max
pulse. Initial pulses are the same numbers as initialPulse[] in the sketch
and are written as soon as this program starts.

Connections (check these before powering the servos)
----------------------------------------------------
PCA9685 logic talks to the Pi on I2C bus 1. Servo motor current does NOT
come from the Pi.

  PCA9685 pin     Raspberry Pi 40-pin header
  -----------     --------------------------
  GND             Pin 6    GND          (any Pi GND pin works)
  VCC             Pin 1    3.3 V        chip + I2C logic only
  SDA             Pin 3    GPIO 2       I2C1 SDA   (/dev/i2c-1)
  SCL             Pin 5    GPIO 3       I2C1 SCL   (/dev/i2c-1)

  PCA9685 V+      external 5–6 V servo supply (+)
  PCA9685 GND     that supply's (−), tied to the same Pi GND above

  One common ground: Pi GND, PCA9685 GND, and the servo supply (−).

Servo plugs, headers marked with the channel number (0–15):

  motor 0 -> channel 10
  motor 1 -> channel 11
  motor 2 -> channel 12
  motor 3 -> channel 13
  motor 4 -> channel 14
  motor 5 -> channel 15

  brown or black          -> GND
  red                     -> V+   (the external servo rail, not Pi 5 V)
  orange, yellow, white   -> PWM  (signal)

Address pads A0–A5 left open: I2C address stays 0x40.
OE left unconnected (active-low output enable; open means outputs on).

Do not power the servos from Pi pin 2 or pin 4 (5 V).

One-time setup on the Pi (same venv as servo_ch10.py)
------------------------------------------------------
  sudo raspi-config          # Interface Options -> I2C -> Enable, then reboot
  sudo apt update
  sudo apt install -y i2c-tools python3-pip python3-venv \
      swig python3-dev build-essential liblgpio-dev
  i2cdetect -y 1             # a "40" in the grid means the board answered

  python3 -m venv .venv && . .venv/bin/activate
  pip install adafruit-circuitpython-servokit

Run
---
  python3 arm_servos.py                 # snap to initial pulses, then prompt
  python3 arm_servos.py --soften 40     # 40 ms between each 1 deg step
  python3 arm_servos.py 0 120           # motor 0 soft-moves to 120 deg, then exit

At the prompt
-------------
  0 120          motor 0 to 120 deg, 1 deg at a time
  soften 40      ms to wait after each 1 deg step (0 jumps straight there)
  status
  home           soft-move every motor back to its initial pulse
  q
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

# Same PCA9685 address and frame rate as esp32_slider.ino.
I2C_ADDRESS = 0x40
PWM_FREQUENCY_HZ = 50
TRAVEL_DEG = 180.0

# Milliseconds to wait between 1-degree steps. Higher is slower.
# 0 writes the target once.
DEFAULT_SOFTEN_MS = 20.0


@dataclass(frozen=True)
class Motor:
    channel: int
    min_us: int
    max_us: int
    initial_us: int


# Copied from esp32_slider.ino. Do not change these to "clean them up".
# motor 0..5 in this order.
MOTORS = (
    Motor(10, 500, 2900, 1700),
    Motor(11, 500, 2700, 900),
    Motor(12, 500, 2900, 1700),
    Motor(13, 500, 2900, 2000),
    Motor(14, 500, 2900, 2050),
    Motor(15, 1100, 1600, 1550),
)


def us_to_deg(min_us: int, max_us: int, pulse_us: float) -> float:
    """Map a pulse inside [min_us, max_us] onto 0..180 deg."""
    return (pulse_us - min_us) * TRAVEL_DEG / (max_us - min_us)


def deg_to_us(min_us: int, max_us: int, degrees: float) -> float:
    """Inverse of us_to_deg. 0 deg -> min_us, 180 deg -> max_us."""
    return min_us + (max_us - min_us) * (degrees / TRAVEL_DEG)


def commanded_us(motor: Motor, degrees: float) -> int:
    return int(round(deg_to_us(motor.min_us, motor.max_us, degrees)))


def angles_along(current: float, target: float) -> list[float]:
    """1-degree steps from current toward target, ending on target.

    The start angle is not included. An empty list means already there.
    The last step is shorter than 1 deg when the gap is not a whole number.
    """
    if math.isclose(current, target, abs_tol=1e-6):
        return []
    direction = 1.0 if target > current else -1.0
    cursor = current
    path = []
    while abs(target - cursor) > 1.0:
        cursor += direction
        path.append(cursor)
    path.append(target)
    return path


def initial_deg(motor: Motor) -> float:
    return us_to_deg(motor.min_us, motor.max_us, motor.initial_us)


class Arm:
    def __init__(self, kit, soften_ms: float) -> None:
        self._kit = kit
        self.soften_ms = soften_ms
        self.deg = [initial_deg(motor) for motor in MOTORS]

    def snap_initial(self) -> None:
        """Write every initial pulse immediately, same as the sketch's setup()."""
        print("initial positions (pulses unchanged from esp32_slider.ino):")
        print("0 deg = that motor's min pulse, 180 deg = its max pulse")
        for index, _motor in enumerate(MOTORS):
            self._write(index, self.deg[index])
            self._report(index, final=True)

    def home(self) -> None:
        for index, _motor in enumerate(MOTORS):
            self.move_to(index, self.deg_initial(index))

    def deg_initial(self, index: int) -> float:
        return initial_deg(MOTORS[index])

    def move_to(self, index: int, degrees: float) -> None:
        if degrees < -1e-6 or degrees > TRAVEL_DEG + 1e-6:
            print(f"clamped to 0..{TRAVEL_DEG:.0f} deg")
        degrees = min(TRAVEL_DEG, max(0.0, degrees))
        if self.soften_ms <= 0:
            path = (
                []
                if math.isclose(self.deg[index], degrees, abs_tol=1e-6)
                else [degrees]
            )
            delay_s = 0.0
        else:
            path = angles_along(self.deg[index], degrees)
            delay_s = self.soften_ms / 1000.0
        if not path:
            self._report(index, final=True)
            return
        for step, angle in enumerate(path):
            self._write(index, angle)
            self.deg[index] = angle
            last = step == len(path) - 1
            self._report(index, final=last)
            if delay_s and not last:
                time.sleep(delay_s)

    def print_status(self) -> None:
        print(f"soften {self.soften_ms:g} ms/deg")
        for index, motor in enumerate(MOTORS):
            us = commanded_us(motor, self.deg[index])
            init = motor.initial_us
            print(
                f"motor {index}  ch {motor.channel:2d}  "
                f"{self.deg[index]:6.2f} deg ({us:4d} us)  "
                f"initial {init:4d} us  "
                f"limits {motor.min_us}-{motor.max_us} us"
            )

    def _write(self, index: int, degrees: float) -> None:
        channel = MOTORS[index].channel
        self._kit.servo[channel].angle = degrees

    def _report(self, index: int, final: bool) -> None:
        motor = MOTORS[index]
        us = commanded_us(motor, self.deg[index])
        text = (
            f"motor {index} -> ch {motor.channel} -> "
            f"{self.deg[index]:6.2f} deg ({us:4d} us)"
        )
        print("\r" + text, end="\n" if final else "", flush=True)


def _kit():
    try:
        from board import SCL, SDA
        import busio
        from adafruit_servokit import ServoKit
    except ImportError as exc:
        raise SystemExit(
            "Missing library. On the Pi, in the ARM_control venv:\n"
            "  pip install adafruit-circuitpython-servokit\n"
            f"Import error: {exc}"
        ) from exc

    i2c = busio.I2C(SCL, SDA)
    kit = ServoKit(
        channels=16,
        i2c=i2c,
        address=I2C_ADDRESS,
        frequency=PWM_FREQUENCY_HZ,
    )
    for motor in MOTORS:
        servo = kit.servo[motor.channel]
        servo.actuation_range = int(TRAVEL_DEG)
        servo.set_pulse_width_range(motor.min_us, motor.max_us)
    # Sketch waits 500 ms after pwm.begin() before the first pulses.
    time.sleep(0.5)
    return kit


def print_help() -> None:
    print(
        "commands:\n"
        "  <motor 0-5> <degrees 0-180>   step 1 deg at a time\n"
        "  soften <ms>                   wait this long between steps (0 = jump)\n"
        "  status                        angles, pulses, limits\n"
        "  home                          back to the initial pulses, softened\n"
        "  help\n"
        "  q"
    )


def handle_line(arm: Arm, line: str) -> bool:
    parts = line.split()
    if not parts:
        return True
    cmd = parts[0].lower()
    if cmd in {"q", "quit", "exit"}:
        return False
    if cmd in {"help", "?"}:
        print_help()
        return True
    if cmd == "status":
        arm.print_status()
        return True
    if cmd == "home":
        try:
            arm.home()
        except KeyboardInterrupt:
            print("\nstopped")
        return True
    if cmd == "soften":
        if len(parts) != 2:
            print(f"soften is {arm.soften_ms:g} ms/deg. usage: soften <ms>")
            return True
        try:
            value = float(parts[1])
        except ValueError:
            print("soften wants a number of milliseconds, e.g. soften 40")
            return True
        if value < 0:
            print("soften must be >= 0")
            return True
        arm.soften_ms = value
        print(f"soften {arm.soften_ms:g} ms/deg")
        return True
    if len(parts) != 2:
        print("usage: <motor 0-5> <degrees>   | soften <ms> | status | home | q")
        return True
    try:
        motor = int(parts[0])
        degrees = float(parts[1])
    except ValueError:
        print("usage: <motor 0-5> <degrees>")
        return True
    if not 0 <= motor < len(MOTORS):
        print("motor must be 0-5")
        return True
    try:
        arm.move_to(motor, degrees)
    except KeyboardInterrupt:
        print("\nstopped")
    return True


def repl(arm: Arm) -> None:
    print(
        f"soften {arm.soften_ms:g} ms/deg "
        "(soften <ms> to change, 0 jumps). type help."
    )
    while True:
        try:
            line = input(f"arm (soften {arm.soften_ms:g} ms/deg)> ").strip()
        except EOFError:
            print()
            return
        except KeyboardInterrupt:
            print()
            return
        if not handle_line(arm, line):
            return


def _check_tables() -> None:
    """Fail fast if an edit knocks an initial pulse outside its limits."""
    if len(MOTORS) != 6:
        raise RuntimeError("expected 6 motors")
    for index, motor in enumerate(MOTORS):
        if motor.max_us <= motor.min_us:
            raise RuntimeError(f"motor {index} max pulse must be above min")
        if not motor.min_us <= motor.initial_us <= motor.max_us:
            raise RuntimeError(
                f"motor {index} initial {motor.initial_us} us "
                f"is outside {motor.min_us}-{motor.max_us}"
            )
        # The degree we store must round-trip to the same integer microsecond.
        us = commanded_us(motor, initial_deg(motor))
        if us != motor.initial_us:
            raise RuntimeError(
                f"motor {index} initial {motor.initial_us} us "
                f"maps back to {us} us"
            )


def main(argv: list[str] | None = None) -> int:
    _check_tables()
    parser = argparse.ArgumentParser(
        description=(
            "Drive the 6 arm servos on PCA9685 channels 10-15. "
            "Angles are degrees across each motor's own pulse range."
        )
    )
    parser.add_argument("motor", nargs="?", type=int, help="motor 0-5")
    parser.add_argument(
        "degrees",
        nargs="?",
        type=float,
        help="target angle, 0-180",
    )
    parser.add_argument(
        "--soften",
        type=float,
        default=DEFAULT_SOFTEN_MS,
        help=(
            "milliseconds between each 1 deg step "
            f"(default {DEFAULT_SOFTEN_MS:g}, 0 jumps)"
        ),
    )
    args = parser.parse_args(argv)
    if (args.motor is None) != (args.degrees is None):
        parser.error("pass both motor and degrees, or neither for the prompt")
    if args.motor is not None and not 0 <= args.motor < len(MOTORS):
        parser.error("motor must be 0-5")
    if args.soften < 0:
        parser.error("soften must be >= 0")

    arm = Arm(_kit(), args.soften)
    arm.snap_initial()
    try:
        if args.motor is None:
            repl(arm)
        else:
            arm.move_to(args.motor, args.degrees)
    except KeyboardInterrupt:
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
