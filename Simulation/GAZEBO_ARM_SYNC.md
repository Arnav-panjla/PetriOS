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

### Direct Gazebo transfer (no GUI)

This is the quickest way to inspect the complete pickup and placement. Stop
any older Gazebo launch first, then run:

```bash
cd /home/manashvi/PetriOS
source /opt/ros/humble/setup.bash
colcon build --base-paths Simulation --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch amr_simulation gazebo_petri_direct.launch.py speed:=1.5
```

The new launch reads the saved home and IK calibration directly, spawns the
carousel dish and camera housing, and commands Gazebo's arm and gripper
controllers without a web GUI or physical motor driver. It groups path
waypoints into timed ROS trajectories and aligns the held dish horizontally
with the camera shelf before release. `speed:=1.0` runs more slowly;
`speed:=2.0` runs faster. Stop the launch and start it again to reset the dish.
The terminal prints each stage and the final dish center error. Do not run a
GUI-sync Gazebo launch in the same ROS domain at the same time.

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
enters the U slot, grips and lifts the dish, extracts it clear of the carousel,
travels level to above the camera housing, lowers vertically through the
collision-disabled upper plate into the inner U slot, releases, retracts
vertically, and returns home. The upper plate remains visible in Gazebo but is
not part of its collision mesh. Use the speed selector to adjust motion.
The selector now defaults to 3× and offers 4× and 6× for faster simulation;
hardware mode is capped at 3×. Gazebo mirror commands use a 50 ms duration to
track the faster GUI updates more closely.
The carousel and dish have been moved 50 mm radially inward; the camera housing
has been moved a further 80 mm inward from the previous scene. The dish route
uses the opening through the collision-disabled upper plate.

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
