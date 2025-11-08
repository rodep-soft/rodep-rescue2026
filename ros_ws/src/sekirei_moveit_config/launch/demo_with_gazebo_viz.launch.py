#!/usr/bin/env python3
"""Launch Mock Hardware with Gazebo visualization

This launches:
- Mock Hardware (ros2_control)
- MoveIt Move Group
- Gazebo for visualization only (no physics simulation)
- Bridge to sync joint states to Gazebo

The robot is controlled by Mock Hardware, and Gazebo just displays it.
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_moveit = get_package_share_directory('sekirei_moveit_config')

    # Start demo with Mock Hardware (no RViz)
    demo_launch = ExecuteProcess(
        cmd=[
            'ros2', 'launch',
            os.path.join(pkg_moveit, 'launch', 'demo.launch.py'),
            'use_rviz:=false'
        ],
        output='screen',
    )

    # Start Gazebo (headless server for visualization)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r'],
        output='screen',
    )

    # Spawn robot model in Gazebo (visualization only)
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'sekirei_arm',
                    '-topic', 'robot_description',
                ],
                output='screen',
            )
        ]
    )

    # Bridge joint states from ros2_control to Gazebo
    # This syncs the Mock Hardware joint positions to Gazebo visualization
    joint_state_bridge = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                    '--ros-args', '-r', '/joint_states:=/joint_states_gz'
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        demo_launch,
        gazebo,
        spawn_robot,
        # joint_state_bridge,  # Disabled for now, might not be needed
    ])
