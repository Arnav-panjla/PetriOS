import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg_amr_simulation = get_package_share_directory('amr_simulation')
    pkg_arm = get_package_share_directory('robot_arm_description')
    pkg_carousel = get_package_share_directory('carousel_description')
    pkg_camera = get_package_share_directory('camera_housing')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Build comprehensive resource path list for Ignition Gazebo mesh resolution
    resource_paths = [
        os.path.dirname(pkg_arm),
        os.path.dirname(pkg_carousel),
        os.path.dirname(pkg_camera),
        os.path.dirname(pkg_amr_simulation),
        os.path.abspath(os.path.join(pkg_amr_simulation, '../../../../src'))
    ]

    ign_existing = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    gz_existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    if ign_existing:
        resource_paths.append(ign_existing)
    if gz_existing:
        resource_paths.append(gz_existing)

    resource_path_str = ':'.join(resource_paths)

    set_ign_env = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path_str)
    set_gz_env = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path_str)

    # Path to Xacro file and World file
    xacro_file = os.path.join(pkg_amr_simulation, 'urdf', 'amr_system.urdf.xacro')
    world_file = os.path.join(pkg_amr_simulation, 'worlds', 'amr_stage1.world')

    # Process Xacro file into URDF XML string
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_xml = robot_description_config.toxml()

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': True
        }]
    )

    # Ignition Gazebo / Gazebo Sim Launch Include (gz_sim.launch.py)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[('gz_args', f'-r {world_file}')]
    )

    # Spawn Entity Node in Ignition / Gazebo Sim (ros_gz_sim create)
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'amr_system',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_ign_env,
        set_gz_env,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity_node
    ])
