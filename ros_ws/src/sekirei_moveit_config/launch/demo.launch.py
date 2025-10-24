import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    
    # MoveIt Cpp config (planning pipelines)
    moveit_cpp_yaml = load_yaml('sekirei_moveit_config', 'config/moveit_cpp.yaml')



    # Load moveit_controllers.yaml for fake execution
    moveit_controllers = load_yaml('sekirei_moveit_config', 'config/moveit_controllers.yaml')

    # Trajectory execution and moveit_controller_manager parameters
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # Fake execution type - how to animate trajectory execution
    fake_execution_params = {
        'moveit_fake_controller_manager.fake_execution_type': 'interpolate',  # or 'via_points' or 'last_point'
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
    }
    
    # Add MoveIt Cpp configuration (planning pipelines)
    if moveit_cpp_yaml:
        # Convert pipeline_names list to ParameterValue
        if 'planning_pipelines' in moveit_cpp_yaml:
            pp = moveit_cpp_yaml['planning_pipelines']
            if 'pipeline_names' in pp and isinstance(pp['pipeline_names'], list):
                pp['pipeline_names'] = ParameterValue(pp['pipeline_names'])
        
        # Convert ompl.planning_plugins list to ParameterValue
        if 'ompl' in moveit_cpp_yaml and isinstance(moveit_cpp_yaml['ompl'], dict):
            ompl_config = moveit_cpp_yaml['ompl']
            if 'planning_plugins' in ompl_config and isinstance(ompl_config['planning_plugins'], list):
                ompl_config['planning_plugins'] = ParameterValue(ompl_config['planning_plugins'])
        
        move_group_params.update(moveit_cpp_yaml)

    # Add OMPL planning configuration
    if ompl_planning_yaml:
        # Convert planning_plugins list to ParameterValue
        if 'planning_plugins' in ompl_planning_yaml and isinstance(ompl_planning_yaml['planning_plugins'], list):
            ompl_planning_yaml['planning_plugins'] = ParameterValue(ompl_planning_yaml['planning_plugins'])
        
        # Convert request_adapters list to ParameterValue
        if 'request_adapters' in ompl_planning_yaml and isinstance(ompl_planning_yaml['request_adapters'], list):
            ompl_planning_yaml['request_adapters'] = ParameterValue(ompl_planning_yaml['request_adapters'])
        
        # Convert planner_configs lists in group-specific config
        if 'sekirei_arm' in ompl_planning_yaml and isinstance(ompl_planning_yaml['sekirei_arm'], dict):
            if 'planner_configs' in ompl_planning_yaml['sekirei_arm']:
                if isinstance(ompl_planning_yaml['sekirei_arm']['planner_configs'], list):
                    ompl_planning_yaml['sekirei_arm']['planner_configs'] = ParameterValue(
                        ompl_planning_yaml['sekirei_arm']['planner_configs']
                    )
        
        # Add with 'ompl.' prefix to match expected namespace
        for key, value in ompl_planning_yaml.items():
            move_group_params[f'ompl.{key}'] = value

    # Add trajectory execution parameters
    move_group_params.update(trajectory_execution)
    move_group_params.update(fake_execution_params)
    move_group_params.update(planning_scene_monitor_parameters)

    # Add controller configurations from moveit_controllers.yaml
    if moveit_controllers:
        # Convert controller_names list to ParameterValue if needed
        if 'moveit_simple_controller_manager' in moveit_controllers:
            scm = moveit_controllers['moveit_simple_controller_manager']
            if 'controller_names' in scm and isinstance(scm['controller_names'], list):
                scm['controller_names'] = ParameterValue(scm['controller_names'])
        move_group_params.update(moveit_controllers)
    
    # Fake controller - Python action server that publishes joint states
    fake_controller_node = Node(
        package='sekirei_moveit_config',
        executable='fake_controller.py',
        output='screen',
        name='fake_controller',
    )
    
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

    # Don't use joint_state_publisher - fake controller will publish joint states
    # The fake controller action server will publish to /joint_states directly

    nodes_to_start = [
        static_tf_node,
        robot_state_publisher,
        fake_controller_node,
        move_group_node,
        rviz_node,
    ]

    return nodes_to_start


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
