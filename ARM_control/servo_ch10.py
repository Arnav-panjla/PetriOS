"""
Drive one hobby servo on PCA9685 channel 10 from a Raspberry Pi over I2C.

Connections (check these before powering the servo)
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

Servo plug, on the screw terminal / header marked 10 (channels are 0–15):

  brown or black          -> GND
  red                     -> V+   (the external servo rail, not Pi 5 V)
  orange, yellow, white   -> PWM  (signal)

Address pads A0–A5 left open: I2C address stays 0x40.
OE left unconnected (active-low output enable; open means outputs on).

Do not power the servo from Pi pin 2 or pin 4 (5 V). A stall browns out the Pi.

One-time setup on the Pi
------------------------
  sudo raspi-config          # Interface Options -> I2C -> Enable, then reboot
  sudo apt install -y i2c-tools python3-pip python3-venv
  i2cdetect -y 1             # a "40" in the grid means the board answered

  python3 -m venv .venv && . .venv/bin/activate
  pip install adafruit-circuitpython-servokit

Run
---
  python3 servo_ch10.py 90          # hold 90 degrees
  python3 servo_ch10.py --sweep     # 0 -> 180 -> 0, then hold 90
  python3 servo_ch10.py             # type angles until you enter q
"""

from __future__ import annotations

import argparse
import sys
import time

# Channel printed on the PCA9685 header. 0-indexed, so this is the 11th output.
SERVO_CHANNEL = 10

# Default Adafruit PCA9685 address with A0–A5 unsoldered.
I2C_ADDRESS = 0x40

# Hobby-servo frame rate. Period is 20 ms.
PWM_FREQUENCY_HZ = 50

# Pulse width at the ends of travel. SG90 / MG90S class is often ~500–2500 µs
# for 0–180°. If the horn stops early or buzzes at the end, narrow this range.
MIN_PULSE_US = 500
MAX_PULSE_US = 2500
ACTUATION_RANGE_DEG = 180


def _kit():
    try:
        from board import SCL, SDA
        import busio
        from adafruit_servokit import ServoKit
    except ImportError as exc:
        raise SystemExit(
            "Missing library. On the Pi, in a venv:\n"
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
    servo = kit.servo[SERVO_CHANNEL]
    servo.actuation_range = ACTUATION_RANGE_DEG
    servo.set_pulse_width_range(MIN_PULSE_US, MAX_PULSE_US)
    return kit


def set_angle(kit, degrees: float) -> None:
    if not 0 <= degrees <= ACTUATION_RANGE_DEG:
        raise ValueError(
            f"angle must be 0..{ACTUATION_RANGE_DEG}, got {degrees}"
        )
    kit.servo[SERVO_CHANNEL].angle = degrees
    print(f"channel {SERVO_CHANNEL}: {degrees:.1f} deg")


def sweep(kit, step_deg: float = 10, dwell_s: float = 0.15) -> None:
    """Move end to end so you can see the horn before commanding a pose."""
    points = []
    angle = 0.0
    while angle < ACTUATION_RANGE_DEG:
        points.append(angle)
        angle += step_deg
    points.append(float(ACTUATION_RANGE_DEG))
    points.extend(points[-2::-1])
    for degrees in points:
        set_angle(kit, degrees)
        time.sleep(dwell_s)


def interactive(kit) -> None:
    print(
        f"Channel {SERVO_CHANNEL}. Enter degrees 0–{ACTUATION_RANGE_DEG}, "
        "or q to stop PWM and exit."
    )
    while True:
        try:
            line = input("angle> ").strip()
        except EOFError:
            print()
            break
        if line.lower() in {"q", "quit", "exit"}:
            break
        if not line:
            continue
        try:
            set_angle(kit, float(line))
        except ValueError as exc:
            print(exc)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            f"Set the servo on PCA9685 channel {SERVO_CHANNEL} "
            f"(I2C 0x{I2C_ADDRESS:02X})."
        )
    )
    parser.add_argument(
        "angle",
        nargs="?",
        type=float,
        help=f"hold this angle, 0–{ACTUATION_RANGE_DEG}",
    )
    parser.add_argument(
        "--sweep",
        action="store_true",
        help="sweep 0 to 180 and back, then hold 90",
    )
    parser.add_argument(
        "--release",
        action="store_true",
        help="turn channel PWM off on exit so the servo goes limp",
    )
    args = parser.parse_args(argv)

    kit = _kit()
    try:
        if args.sweep:
            sweep(kit)
            set_angle(kit, 90)
        elif args.angle is not None:
            set_angle(kit, args.angle)
        else:
            interactive(kit)
    except KeyboardInterrupt:
        print()
    finally:
        if args.release:
            kit.servo[SERVO_CHANNEL].angle = None
            print(f"channel {SERVO_CHANNEL}: PWM off")
    return 0


if __name__ == "__main__":
    sys.exit(main())
