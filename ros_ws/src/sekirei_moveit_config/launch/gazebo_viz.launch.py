#!/usr/bin/env python3
"""Launch with Gazebo visualization + Mock Hardware control

Gazebo shows the robot moving but Mock Hardware does the control.
This is simpler than full physics simulation.
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    """Load yaml file"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f'Error loading yaml file {absolute_file_path}: {e}')
        return {}


def generate_launch_description():
    pkg_moveit = get_package_share_directory('sekirei_moveit_config')
    pkg_urdf = get_package_share_directory('urdf_test_node')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # === 1. Robot Description ===
    urdf_path = os.path.join(pkg_urdf, 'urdf', 'sekirei.urdf')
    robot_description = ParameterValue(
        Command(['cat ', urdf_path]),
        value_type=str
    )

    # === 2. Semantic Description (SRDF) ===
    srdf_path = os.path.join(pkg_moveit, 'config', 'sekirei.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = f.read()

    # === 3. Kinematics ===
    kinematics_yaml = load_yaml('sekirei_moveit_config', 'config/kinematics.yaml')

    # === 4. Planning ===
    ompl_yaml = load_yaml('sekirei_moveit_config', 'config/ompl_planning.yaml')

    # === 5. Controllers ===
    controllers_yaml = load_yaml('sekirei_moveit_config', 'config/ros2_controllers.yaml')

    # === Robot State Publisher ===
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    # === ROS2 Control Node (Mock Hardware) ===
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml,
        ],
        output='screen',
    )

    # === Controller Spawner ===
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['sekirei_arm_controller', '--controller-manager', '/controller_manager'],
    )

    # === Move Group ===
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_yaml,
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'request_adapters': 'default_planning_request_adapters/ResolveConstraintFrames '
                                   'default_planning_request_adapters/ValidateWorkspaceBounds '
                                   'default_planning_request_adapters/CheckStartStateBounds '
                                   'default_planning_request_adapters/CheckStartStateCollision '
                                   'planning_request_adapter_plugins/AddTimeOptimalParameterization',
                'start_state_max_bounds_error': 0.1,
                'jiggle_fraction': 0.05,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
                'use_sim_time': use_sim_time,
            },
            ompl_yaml,
        ],
    )

    # === Start Gazebo ===
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4'],
        output='screen',
    )

    # === Spawn Robot in Gazebo (after delay) ===
    gazebo_urdf_path = os.path.join(pkg_urdf, 'urdf', 'sekirei_gazebo.urdf')
    with open(gazebo_urdf_path, 'r') as f:
        gazebo_robot_description = f.read()

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-name', 'sekirei',
                    '-string', gazebo_robot_description,
                    '-x', '0', '-y', '0', '-z', '0.5',
                ],
                output='screen',
            )
        ]
    )

    # === Bridge joint states to Gazebo (for visualization) ===
    joint_state_bridge = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        start_gazebo,
        spawn_robot,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        # joint_state_bridge,  # Enable if needed
    ])
