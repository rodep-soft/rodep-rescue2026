"""Launch ros2_control node for sekirei robot using dynamixel_hardware.

This launch file:
 - loads the URDF (`urdf/sekirei_moveit.urdf`) as robot_description
 - starts robot_state_publisher
 - starts `ros2_control_node` (controller_manager) which reads the <ros2_control> block in URDF
 - spawns `joint_state_broadcaster` and `joint_trajectory_controller`

Usage:
  colcon build --symlink-install
  source install/setup.fish
  ros2 launch sekirei_moveit_config dynamixel_hw.launch.py

Note: The URDF contains hardware params (usb_port, baud_rate, joint ids). Edit the URDF or set use_dummy to true for dry-run.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import os
import shutil


def generate_launch_description():
    pkg_share = FindPackageShare(package='sekirei_moveit_config')
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'sekirei_moveit.urdf'])

    # robot_description parameter: load the URDF file content directly (avoid shell Command issues)
    # Locate the URDF in the installed package shares. Try urdf_test_node then sekirei_moveit_config.
    robot_description_content = ''
    urdf_file = ''
    try:
        urdf_file = os.path.join(get_package_share_directory('urdf_test_node'), 'urdf', 'sekirei_moveit.urdf')
    except PackageNotFoundError:
        try:
            urdf_file = os.path.join(get_package_share_directory('sekirei_moveit_config'), 'urdf', 'sekirei_moveit.urdf')
        except PackageNotFoundError:
            urdf_file = ''

    if urdf_file:
        try:
            with open(urdf_file, 'r') as f:
                robot_description_content = f.read()
        except Exception:
            robot_description_content = ''

    robot_description = robot_description_content

    # robot_state_publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # ros2_control_node (controller_manager) - only launch if available
    controller_nodes = []
    try:
        # Check whether controller_manager/ros2_control_node is installed
        _ = get_package_share_directory('controller_manager')
        controller_nodes.append(
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                parameters=[{'robot_description': robot_description}],
                output='screen'
            )
        )

        # Spawner nodes for controllers
        controller_nodes.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen'
            )
        )

        controller_nodes.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_trajectory_controller'],
                output='screen'
            )
        )
    except PackageNotFoundError:
        print("[launch warning] controller_manager package not found; skipping ros2_control_node and spawners. Install ros2_control/ros2_controllers to enable hardware controllers.")

    # combine nodes: always start robot_state_publisher, add controller nodes only if available
    nodes = [rsp_node] + controller_nodes
    return LaunchDescription(nodes)
