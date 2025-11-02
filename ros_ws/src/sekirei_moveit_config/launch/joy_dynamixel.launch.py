"""Launch dynamixel hardware stack + joystick teleop

This starts:
 - dynamixel_hw (ros2_control_node via dynamixel_hw.launch.py)
 - joystick node (joy_node)
 - joy_teleop node (this script)

Usage:
  source install/setup.fish
  ros2 launch sekirei_moveit_config joy_dynamixel.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('sekirei_moveit_config')
    dynamixel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'dynamixel_hw.launch.py'))
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }]
    )

    joy_teleop = Node(
        package='sekirei_moveit_config',
        executable='joy_teleop.py',
        name='joy_teleop',
        output='screen',
        # For systems without controller_manager installed, use the JointState publisher fallback
        parameters=[{'use_action': False}],
    )

    # Use the minimal RViz config (no MoveIt plugins) to avoid SRDF/action server dependencies
    rviz_config = os.path.join(pkg_share, 'config', 'rviz_simple.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        # Clear any RViz config cache/defaults
        additional_env={'RVIZ2_CONFIG': rviz_config}
    )

    # Delay RViz start to give robot_state_publisher time to publish robot_description
    delayed_rviz = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg=f'Starting RViz with config: {rviz_config}'),
            rviz_node
        ]
    )

    return LaunchDescription([
        dynamixel_launch,
        joy_node,
        joy_teleop,
        delayed_rviz,
    ])
