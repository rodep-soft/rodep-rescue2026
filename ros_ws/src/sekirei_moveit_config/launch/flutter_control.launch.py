#!/usr/bin/env python3
"""
Complete Launch for Flutter UI Control

This launches:
1. MoveIt (with Mock Hardware or real Dynamixel)
2. rosbridge_server for Flutter connection
3. moveit_bridge node for simple pose commands
4. RViz2 for visualization

Usage:
    ros2 launch sekirei_moveit_config flutter_control.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_moveit = get_package_share_directory('sekirei_moveit_config')

    # Launch arguments
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # Include demo.launch.py (with rosbridge enabled by default)
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, 'launch', 'demo.launch.py')
        ),
        launch_arguments={
            'use_rosbridge': 'true',
        }.items()
    )

    # MoveIt Bridge node for Flutter control
    moveit_bridge_node = Node(
        package='sekirei_moveit_config',
        executable='moveit_bridge.py',
        name='moveit_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 for visualization'
        ),
        demo_launch,
        moveit_bridge_node,
    ])
