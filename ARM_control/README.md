# Six-servo arm

Open-loop control of six hobby servos on a PCA9685, from a Raspberry Pi.
The Pi commands pulse width. The servos do not report where they actually are.

The package lives in `ARM_control/robotic-arm`. The ESP32 sketch
`ARM_control/esp32_slider.ino` and the single-servo bring-up script
`ARM_control/servo_ch10.py` are unchanged.

## Layout

- `configs/joints.yaml` is the home pose. The program never writes it.
- `configs/hardware.yaml` is the I2C bus, address, PWM rate, and wiring.
- `configs/motion.yaml` is the uncalibrated speed limits used to time a move.
- `src/robot_arm/hardware` talks to the PCA9685, or records pulses in tests.
- `src/robot_arm/control` is the arm, one joint, and the startup gate.
- `src/robot_arm/motion` is the shared quintic trajectory.
- `configs/kinematics.yaml` and `models/robot.urdf` are the arm geometry
  taken from the simulation URDF. Carousel, camera stand, and dishes are not in them.
- `kinematics`, `vision`, and `simulation` do not solve poses yet. The geometry
  file is what inverse kinematics should use.

## Calibration

Pulses are copied from `esp32_slider.ino`. For each joint, 0° is `min_us`
and 180° is `max_us`.

| Joint | Channel | Min µs | Max µs | Home µs |
|------:|--------:|-------:|-------:|--------:|
| 0 | 10 | 500 | 2900 | 1700 |
| 1 | 11 | 500 | 2700 | 900 |
| 2 | 12 | 500 | 2900 | 1700 |
| 3 | 13 | 500 | 2900 | 2000 |
| 4 | 14 | 500 | 2900 | 2050 |
| 5 | 15 | 1100 | 1600 | 1239 |

Home angles computed from those pulses are 90°, 32.73°, 90°, 112.5°,
116.25°, and 50.04°.

## Cylindrical coordinates

`configs/geometry.yaml` names the servo roles and repeats the URDF joint
origins so they can be checked. The loader reads `models/robot.urdf` and
rejects the YAML if those origins disagree. `configs/calibration.yaml`
holds zero offsets and direction signs. Those start at an identity map.
They are not measured.

`r` and `z` are millimetres about the base joint axis. Positive `r` is the
direction the CAD arm reaches when joint 1 is at its geometric zero.
`theta` and `tool_pitch` are degrees. `q = 0` in the URDF is not the servo
home pulse.

```text
ik 120 45 80 -90
goto 120 45 80 -90
pose
calibration show
calibration set servo_1.zero_offset 5
```

`python -m robot_arm.interfaces.cli --dry-run` prints those results without
sending pulses.

Speed limits in `motion.yaml` (60 °/s and 200 °/s²) were not measured.
Change them there. They are not part of the servo calibration.

## Wiring

PCA9685 logic uses the Pi's I2C header. Servo current comes from a separate
5–6 V supply. Tie that supply's negative to Pi GND.

| PCA9685 | Raspberry Pi |
|---|---|
| GND | Pin 6, GND |
| VCC | Pin 1, 3.3 V (logic only) |
| SDA | Pin 3, GPIO 2 (I2C1 SDA) |
| SCL | Pin 5, GPIO 3 (I2C1 SCL) |
| V+ | External 5–6 V servo supply (+) |
| GND | That supply's negative, tied to Pi GND |

On the header for channels 10 through 15: brown/black to GND, red to V+,
signal wire to PWM. Address pads A0–A5 stay open, so the address is `0x40`.
Do not power the servos from Pi pin 2 or pin 4.

## Setup

On the Pi, enable I2C and install the same build tools the single-servo
script needed (`swig`, `python3-dev`, `build-essential`, `liblgpio-dev`).
`i2cdetect -y 1` should show `40`.

```bash
cd ARM_control/robotic-arm
python3 -m venv .venv
. .venv/bin/activate
pip install -e ".[hardware,dev]"
```

Tests do not need the hardware extra:

```bash
pip install -e ".[dev]"
pytest
```

## CLI

Startup does not know the physical pose, so it asks before snapping every
servo to the stored home pulses. That snap is immediate. It is not a
quintic move, and it is not closed-loop homing.

```bash
python -m robot_arm.interfaces.cli
python -m robot_arm.interfaces.cli --yes
python -m robot_arm.interfaces.cli --yes 0 120
```

`--yes` skips the question. The warning is still printed.
`python3 ../arm_servos.py` from `ARM_control` starts the same CLI.

After startup:

```text
0 120
move 0 120
moveall 90 80 110 90 70 90
home
status
config
soften 40
q
```

`soften <ms>` is the old timing knob: the move lasts `|delta| * ms / 1000`
seconds. `soften 0` jumps. Without it, duration comes from `motion.yaml`.

`home` moves from the last commanded angles back to the YAML home pulses
with the same quintic. It does not change the file. `status` prints
commanded angles only.

A move samples every joint, including joints that hold still, at 50 Hz on
one clock. The blend is `s(τ) = 10τ³ - 15τ⁴ + 6τ⁵`. The last sample is the
exact target pulse. Ctrl+C stops the loop at the last pulse that was sent.
That pulse is the new commanded state. The horn may not have reached it.

## Later

These can sit on top of `PWMDriver` and `ArmController` without changing
the servo calibration code:

- a measured URDF, then forward and inverse kinematics
- Cartesian planning beside the joint-space planner
- ROS 2 and `ros2_control`, with a second `PWMDriver`
- Gazebo behind `simulation.ArmBackend`
- a camera, extrinsic calibration, and dish-sampling tasks
- collision checks before `move_all`
- logging beyond the standard `logging` module, and a web dashboard
