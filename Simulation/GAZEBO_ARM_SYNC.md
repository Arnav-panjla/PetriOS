# PetriOS GUI ↔ Gazebo arm synchronization

The web GUI remains the source of truth for servo zero offsets, rotation signs,
the saved home, and the TCP offset. The ROS bridge only reads the GUI API and
publishes Gazebo trajectories; it contains no PCA9685 or motor code.

The original Gazebo implementation is preserved separately and still runs with:

```bash
ros2 launch amr_simulation gazebo_stage2.launch.py
```

That legacy launch continues to use `amr_system.urdf.xacro`,
`robot_arm_gazebo.urdf.xacro`, and `arm_controllers.yaml`. GUI synchronization
uses the corresponding `*_gui_sync` files and does not alter the legacy path.

## Local simulation-only run

### Gazebo scene preview (no GUI)

This is the quickest way to inspect the current fixture positions. Stop any
older Gazebo launch first, then run:

```bash
cd /home/manashvi/PetriOS
source /opt/ros/humble/setup.bash
colcon build --base-paths Simulation --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch amr_simulation gazebo_petri_direct.launch.py
```

The launch reads the saved home and IK calibration directly and spawns the arm,
carousel dish, and camera housing without a web GUI or physical motor driver.
Pickup and release wait for Gazebo to publish the corresponding attached or
detached state before the next arm movement.
To run the full pickup and placement after inspecting the scene, relaunch with:

```bash
ros2 launch amr_simulation gazebo_petri_direct.launch.py run_transfer:=true speed:=3.0
```

For headless Gazebo, add `gz_args:='-r -s'`. Do not run a GUI-sync Gazebo
launch in the same ROS domain at the same time.

The transfer planner now validates the path internally and keeps only the
joint targets that can be joined by the controller's smooth quintic
interpolation without leaving the planned route. The current scene uses
24 commanded targets. J5 uses the recorded hardware
commands: 16 degrees to open before pickup, 134.5 degrees to grip, and
11.5 degrees to release. The recording supplies gripper angles only; its arm
positions belong to a different fixture layout.

### GUI-synchronized run

Build the ROS 2 workspace with Humble's system Python. This matters on machines
where Conda/Miniforge appears first on `PATH`:

```bash
cd /home/manashvi/PetriOS
source /opt/ros/humble/setup.bash
colcon build --base-paths Simulation --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Start the GUI in a Python 3.12+ environment with NumPy, SciPy, and PyYAML:

```bash
cd /home/manashvi/PetriOS/ARM_control
python3 -m venv .venv
source .venv/bin/activate
pip install -e . numpy scipy
python -m robot_arm.interfaces.web_gui --simulation-only --no-camera
```

`--simulation-only` constructs a no-op driver, never the PCA9685 driver. The
page must show **SIMULATION ONLY (motors disabled)**.

In another terminal, launch Gazebo. Launch fails before spawning the robot if
the GUI is unavailable, because an unverified fallback pose is intentionally
not used:

```bash
cd /home/manashvi/PetriOS
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch amr_simulation gazebo_stage2_gui_sync.launch.py
```

For server-only/headless Gazebo:

```bash
ros2 launch amr_simulation gazebo_stage2_gui_sync.launch.py gz_args:='-r -s'
```

Inspect the running controllers and joint feedback:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

Run the end-to-end validator while the simulation-only GUI, bridge, and Gazebo
are running:

```bash
ros2 run arm_gazebo_bridge validate_kinematics
```

The validator refuses to move anything unless `/api/status` says
`simulation_only=true`. It tests saved home, URDF zero, and two additional
poses, prints GUI-vs-Gazebo TCP mismatch in millimetres, requires at most 1 mm,
and restores home.

## GUI on a Raspberry Pi, Gazebo on a workstation

Run the GUI on the Pi. Use `--simulation-only` for a guaranteed motor-free
session; omit it only when intentionally operating the physical arm under the
existing startup confirmation:

```bash
cd /path/to/PetriOS/ARM_control
source .venv/bin/activate
python -m robot_arm.interfaces.web_gui --simulation-only --no-camera
```

On the workstation, create a tunnel and leave it running:

```bash
ssh -N -L 18000:127.0.0.1:8000 pi@PI_ADDRESS
```

Then launch against the tunnel:

```bash
source /opt/ros/humble/setup.bash
source /home/manashvi/PetriOS/install/setup.bash
ros2 launch amr_simulation gazebo_stage2_gui_sync.launch.py gui_url:=http://127.0.0.1:18000
```

Open `http://127.0.0.1:18000` to use the same GUI. For simulation-only remote
validation, pass the same URL:

```bash
ros2 run arm_gazebo_bridge validate_kinematics --ros-args -p gui_url:=http://127.0.0.1:18000
```

## Gripper calibration boundary

The bridge preserves the GUI's current mapping: 10° is open, 167.5° is closed,
and maximum jaw width is 35 mm. It uses the GUI's 45 mm effective finger length,
mirrors the right finger, and applies the reference CAD linkage approximation of
0.81 to the two auxiliary links. The 45 mm length, 35 mm width, 0.81 ratio, and
auxiliary linkage pivots still need physical measurement. The existing Gazebo
auxiliary pivots are intentionally unchanged because they differ from the CAD
reference and have not been physically verified.

## Follow the Petri path from the GUI

Launch `gazebo_stage2_gui_sync.launch.py` with the simulation-only GUI as shown
above. Once the Gazebo bridge is running, expand **Petri transfer** and click
**Follow Petri path**. The path runs through the GUI's normal `ArmController`:
the browser's 3D model follows its commanded servo angles, and the bridge mirrors
those angles into Gazebo. The bridge waits for the arm to settle before attaching
or releasing the simulated dish. The route opens the gripper at the carousel,
enters the U slot, grips the dish, moves outward while lifting, and continues
straight until the complete dish clears the carousel plates. It then turns
along the arm side of the carousel,
travels level to above the camera opening, lowers through the opening with the
dish horizontal, slides into the inner U slot at shelf height, releases,
retracts horizontally, moves to one low clearance pose outside the housing, and returns
home. The roof has a 110 mm wide front U opening in both its visual and collision
geometry. Use the speed selector to
adjust motion.
The selector now defaults to 3× and offers 4× and 6× for faster simulation;
hardware mode is capped at 3×. Gazebo mirror commands use a 50 ms duration to
track the faster GUI updates more closely.
The carousel base is at (0.307591398, -0.100, 0) m with +15 degree yaw. Its
dish is at (0.207135256, -0.073082858, 0.2400) m, rotated with the carousel.
The camera housing is 110 mm closer to the arm than the saved working layout;
its shelf center is now (0.06, -0.36, 0.0955) m. The dish descends at
(0.06, -0.325) m, then slides 35 mm into the shelf. The carousel base is
unchanged from the approved position. The earlier layout and motion are saved
under `snapshots/camera_route_before_100mm_20261125_185428/`; the version
immediately before the sparse transfer changes is under
`snapshots/before_sparse_transfer_20261125/`.

For a stationary QR read during CSV playback, select
`session_petri_transfer_24_qr_hold.csv` in the GUI's **Session** section and
press **Play**. Point 1 moves in front of the carousel; point 2 opens the
gripper at the same arm pose and holds for six seconds; point 3 enters the
carousel. The optional `hold_s` column pauses after its row's motion. Older
eight-column recordings still play without a hold. The pause only keeps the
arm stationary; the QR reader must be running separately. A GUI-sync Gazebo
bridge mirrors the arm angles during CSV playback. The CSV's `event` column
asks Gazebo to attach the dish after point 4 grips it and detach after point
21 releases it. Playback waits for the bridge to confirm each event before
moving on. A simulation-only GUI requires a connected bridge to play this
event-enabled CSV; hardware playback can run without Gazebo.

With a normal hardware GUI (without `--simulation-only`), the same button sends
the path's servo angles through the motor controller after the normal startup
confirmation. A running Gazebo bridge will mirror those angles too. The current
path uses the fixture coordinates in `robot_arm.motion.petri_transfer` for the
Gazebo scene; measure and update the source and destination coordinates before
using it on hardware. No hardware motion was used to validate this integration.

After a completed simulation transfer, relaunch Gazebo to reset the dish in the
carousel before running it again.

Coordinates are in `base_link` metres. `outward_xy` points from the holder
toward its opening. `inside_m` is the distance of the resting dish centre inside
that opening. The Gazebo detachable joint is a simulation grasp proxy; it does
not establish reliable physical contact or gripping force. Measure and validate
the actual fixtures and gripper clearances before running the path on hardware.
