import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def launch_setup(context, *args, **kwargs):
    # Get package directories
    moveit_config_pkg = get_package_share_directory('sekirei_moveit_config')
    urdf_pkg = get_package_share_directory('urdf_test_node')
    
    # Load URDF (use MoveIt-compatible version with collision and inertial)
    urdf_file = os.path.join(urdf_pkg, 'urdf', 'sekirei_moveit.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Load SRDF
    srdf_file = os.path.join(moveit_config_pkg, 'config', 'sekirei.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()
    
    # Kinematics config
    kinematics_yaml = load_yaml('sekirei_moveit_config', 'config/kinematics.yaml')
    
    # Planning config
    ompl_planning_yaml = load_yaml('sekirei_moveit_config', 'config/ompl_planning.yaml')
    
    # MoveIt controllers
    moveit_controllers = load_yaml('sekirei_moveit_config', 'config/moveit_controllers.yaml')
    
    # Trajectory execution and moveit_controller_manager parameters
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    moveit_controller_manager = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }
    
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    
    # Combine all parameters for move_group
    move_group_params = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': kinematics_yaml,
        'use_sim_time': False,
        'planning_plugin': 'ompl_interface/OMPLPlanner',
    }
    
    # Configure OMPL pipeline - MoveIt2 expects parameters at root level
    if ompl_planning_yaml:
        # Convert request_adapters list to space-separated string to avoid tuple conversion
        if 'request_adapters' in ompl_planning_yaml and isinstance(ompl_planning_yaml['request_adapters'], list):
            ompl_planning_yaml['request_adapters'] = ' '.join(ompl_planning_yaml['request_adapters'])
        
        # Convert planner_configs lists to space-separated strings
        if 'sekirei_arm' in ompl_planning_yaml:
            if 'planner_configs' in ompl_planning_yaml['sekirei_arm']:
                if isinstance(ompl_planning_yaml['sekirei_arm']['planner_configs'], list):
                    ompl_planning_yaml['sekirei_arm']['planner_configs'] = ' '.join(ompl_planning_yaml['sekirei_arm']['planner_configs'])
        
        move_group_params.update(ompl_planning_yaml)
    
    # Add trajectory execution parameters
    move_group_params.update(trajectory_execution)
    move_group_params.update(moveit_controller_manager)
    move_group_params.update(planning_scene_monitor_parameters)
    
    # Add moveit controllers if available
    if moveit_controllers:
        # Convert controller lists to avoid tuple conversion
        for controller_name, controller_config in moveit_controllers.items():
            if isinstance(controller_config, dict) and 'joints' in controller_config:
                if isinstance(controller_config['joints'], list):
                    controller_config['joints'] = ' '.join(controller_config['joints'])
        
        # Convert controller_names list to space-separated string
        if 'controller_names' in moveit_controllers and isinstance(moveit_controllers['controller_names'], list):
            moveit_controllers['controller_names'] = ' '.join(moveit_controllers['controller_names'])
        
        move_group_params.update(moveit_controllers)
    
    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        namespace='',
        parameters=[move_group_params],
    )
    
    # RViz
    rviz_config_file = os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_yaml,
            }
        ],
    )
    
    # Static TF
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link'],
    )
    
    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_description}
        ],
    )
    
    # Joint State Publisher (for testing without real hardware)
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_gui')),
    )
    
    joint_state_publisher_no_gui = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('use_gui')),
    )
    
    nodes_to_start = [
        static_tf_node,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_no_gui,
        move_group_node,
        rviz_node,
    ]
    
    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Use joint_state_publisher_gui',
        )
    )
    
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
