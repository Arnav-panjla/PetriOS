"""Launch the Petri scene and run its transfer directly in Gazebo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

from arm_gazebo_bridge.direct_config import DIRECT_TCP_M, direct_mapping, load_home


def _launch_setup(context):
    speed = float(LaunchConfiguration("speed").perform(context))
    gz_args = LaunchConfiguration("gz_args").perform(context)

    # The calibration files are read directly. No GUI process or hardware driver
    # participates in this launch.
    mapping = direct_mapping()
    arm_home, gripper_home = mapping.all_positions(load_home())

    pkg_amr = get_package_share_directory("amr_simulation")
    pkg_arm = get_package_share_directory("robot_arm_description")
    pkg_carousel = get_package_share_directory("carousel_description")
    pkg_camera = get_package_share_directory("camera_housing")
    pkg_ros_gz = get_package_share_directory("ros_gz_sim")

    resource_paths = [
        os.path.dirname(pkg_arm),
        os.path.dirname(pkg_carousel),
        os.path.dirname(pkg_camera),
        os.path.dirname(pkg_amr),
        os.path.abspath(os.path.join(pkg_amr, "../../../../src")),
    ]
    for variable in ("IGN_GAZEBO_RESOURCE_PATH", "GZ_SIM_RESOURCE_PATH"):
        existing = os.environ.get(variable, "")
        if existing:
            resource_paths.append(existing)
    resource_path = ":".join(resource_paths)

    mappings = {f"arm_initial_{index + 1}": str(value) for index, value in enumerate(arm_home)}
    mappings.update(
        {
            "gripper_initial_left": str(gripper_home[0]),
            "gripper_initial_right": str(gripper_home[1]),
            "linkage_initial_left": str(gripper_home[2]),
            "linkage_initial_right": str(gripper_home[3]),
            "tcp_x": str(DIRECT_TCP_M[0]),
            "tcp_y": str(DIRECT_TCP_M[1]),
            "tcp_z": str(DIRECT_TCP_M[2]),
        }
    )
    system_file = os.path.join(pkg_amr, "urdf", "amr_system_gui_sync.urdf.xacro")
    robot_description = xacro.process_file(system_file, mappings=mappings).toxml()
    petri_file = os.path.join(pkg_amr, "urdf", "petri_dish.urdf.xacro")
    petri_description = xacro.process_file(petri_file).toxml()
    world_file = os.path.join(pkg_amr, "worlds", "amr_stage1.world")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"{gz_args} {world_file}"}.items(),
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "amr_system", "-topic", "robot_description", "-x", "0", "-y", "0", "-z", "0"],
        output="screen",
    )
    spawn_petri = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-name", "petri_dish_1", "-string", petri_description,
                   "-x", "0.280014", "-y", "-0.280014", "-z", "0.2400"],
    )
    initial_detach = ExecuteProcess(
        cmd=["ign", "topic", "-t", "/petri_dish_1/detach",
             "-m", "ignition.msgs.Empty", "-p", "unused: true"],
        output="screen",
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="screen",
        )

    direct_transfer = Node(
        package="arm_gazebo_bridge",
        executable="gazebo_direct_petri",
        output="screen",
        parameters=[{"speed": speed}],
    )

    return [
        # Ubuntu's ROS Python and SciPy use the system NumPy. A user-site NumPy
        # can be ABI-incompatible with them, so isolate the spawned runner.
        SetEnvironmentVariable("PYTHONNOUSERSITE", "1"),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        robot_state_publisher,
        gazebo,
        TimerAction(period=2.0, actions=[spawn]),
        TimerAction(period=3.0, actions=[spawn_petri]),
        TimerAction(period=4.0, actions=[initial_detach]),
        TimerAction(period=5.0, actions=[spawner("joint_state_broadcaster")]),
        TimerAction(period=6.0, actions=[spawner("arm_controller")]),
        TimerAction(period=7.0, actions=[spawner("gripper_controller")]),
        TimerAction(period=9.0, actions=[direct_transfer]),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("speed", default_value="1.5"),
            DeclareLaunchArgument("gz_args", default_value="-r"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
