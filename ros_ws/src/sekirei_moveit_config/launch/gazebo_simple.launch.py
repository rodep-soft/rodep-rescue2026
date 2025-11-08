#!/usr/bin/env python3
"""Simple Gazebo + MoveIt launch for testing

Launches:
- Gazebo Sim (headless)
- Robot State Publisher
- ros2_control with gz_ros2_control
- Joint State Broadcaster
- Arm Controller
- MoveIt Move Group
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    # Paths
    pkg_moveit = get_package_share_directory('sekirei_moveit_config')
    pkg_urdf = get_package_share_directory('urdf_test_node')

    urdf_path = os.path.join(pkg_urdf, 'urdf', 'sekirei_gazebo.urdf')
    srdf_path = os.path.join(pkg_moveit, 'config', 'sekirei.srdf')

    # Process URDF
    robot_description = {'robot_description': open(urdf_path).read()}

    # Start Gazebo (headless)
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s'],
        output='screen',
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'sekirei',
            '-topic', 'robot_description',
        ],
        output='screen',
    )

    # Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Controller Spawners
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['sekirei_arm_controller'],
        output='screen',
    )

    # Delay arm controller after joint state broadcaster
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller],
        )
    )

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        output='screen',
    )

    return LaunchDescription([
        start_gazebo,
        robot_state_pub,
        spawn_robot,
        static_tf,
        joint_state_broadcaster,
        delay_arm_controller,
    ])
