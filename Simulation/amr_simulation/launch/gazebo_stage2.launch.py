import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():

    # ============================================================
    # PACKAGE DIRECTORIES
    # ============================================================

    pkg_amr_simulation = get_package_share_directory('amr_simulation')
    pkg_arm = get_package_share_directory('robot_arm_description')
    pkg_carousel = get_package_share_directory('carousel_description')
    pkg_camera = get_package_share_directory('camera_housing')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')


    # ============================================================
    # GAZEBO / IGNITION RESOURCE PATH
    # ============================================================

    resource_paths = [
        os.path.dirname(pkg_arm),
        os.path.dirname(pkg_carousel),
        os.path.dirname(pkg_camera),
        os.path.dirname(pkg_amr_simulation),
        os.path.abspath(
            os.path.join(pkg_amr_simulation, '../../../../src')
        )
    ]

    ign_existing = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    gz_existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    if ign_existing:
        resource_paths.append(ign_existing)

    if gz_existing:
        resource_paths.append(gz_existing)

    resource_path_str = ':'.join(resource_paths)

    set_ign_env = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        resource_path_str
    )

    set_gz_env = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        resource_path_str
    )


    # ============================================================
    # FILE PATHS
    # ============================================================

    system_xacro_file = os.path.join(
        pkg_amr_simulation,
        'urdf',
        'amr_system.urdf.xacro'
    )

    petri_xacro_file = os.path.join(
        pkg_amr_simulation,
        'urdf',
        'petri_dish.urdf.xacro'
    )

    world_file = os.path.join(
        pkg_amr_simulation,
        'worlds',
        'amr_stage1.world'
    )


    # ============================================================
    # PROCESS SYSTEM XACRO
    # ============================================================

    system_description_config = xacro.process_file(
        system_xacro_file
    )

    system_description_xml = (
        system_description_config.toxml()
    )


    # ============================================================
    # PROCESS PETRI DISH XACRO
    # ============================================================

    petri_description_config = xacro.process_file(
        petri_xacro_file
    )

    petri_description_xml = (
        petri_description_config.toxml()
    )


    # ============================================================
    # ROBOT STATE PUBLISHER
    # ============================================================

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': system_description_xml,
                'use_sim_time': True
            }
        ]
    )


    # ============================================================
    # GAZEBO SIM
    # ============================================================

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments=[
            (
                'gz_args',
                f'-r {world_file}'
            )
        ]
    )


    # ============================================================
    # SPAWN COMPLETE AMR SYSTEM
    # ============================================================

    spawn_system_node = Node(
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


    # ============================================================
    # ROS 2 CONTROL - JOINT STATE BROADCASTER
    # ============================================================

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )


    # ============================================================
    # ROS 2 CONTROL - ARM TRAJECTORY CONTROLLER
    # ============================================================

    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    spawn_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    # Give Gazebo time to start before spawning controllers
    controller_start = TimerAction(
        period=3.0,
        actions=[
            spawn_joint_state_broadcaster
        ]
    )

    arm_controller_start = TimerAction(
        period=4.0,
        actions=[
            spawn_arm_controller
        ]
    )

    gripper_controller_start = TimerAction(
        period=5.0,
        actions=[
            spawn_gripper_controller
        ]
    )

    # ============================================================
    # SPAWN PETRI DISH
    #
    # Carousel world position:
    #     (0.200000, -0.3464101615)
    #
    # Petri dish position for the upper/front slot:
    #     x = 0.270000
    #     y = -0.3464101615
    #     z = 0.207500
    #
    # ============================================================

    # spawn_petri_node = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=[
    #         '-name', 'petri_dish_1',
    #         '-string', petri_description_xml,
    #         '-x', '0.269565295',
    #         '-y', '-0.278917495',
    #         '-z', '0.240000'
    #     ],
    #     output='screen'
    # )

    # ============================================================
    # LAUNCH DESCRIPTION
    # ============================================================

    return LaunchDescription([
        set_ign_env,
        set_gz_env,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_system_node,
        # spawn_petri_node,
        controller_start,
        arm_controller_start,
        gripper_controller_start
    ])