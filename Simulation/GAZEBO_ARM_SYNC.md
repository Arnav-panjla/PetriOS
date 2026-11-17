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
