import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    """Load yaml file"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    # Get package directories
    pkg_sekirei_moveit = get_package_share_directory('sekirei_moveit_config')
    pkg_urdf_test = get_package_share_directory('urdf_test_node')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    urdf_file = os.path.join(pkg_urdf_test, 'urdf', 'sekirei_gazebo.urdf')
    srdf_file = os.path.join(pkg_sekirei_moveit, 'config', 'sekirei.srdf')
    ros2_controllers_file = os.path.join(pkg_sekirei_moveit, 'config', 'ros2_controllers.yaml')

    # Robot description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_file,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot semantic description
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            srdf_file,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # Gazebo world
    world = LaunchConfiguration('world')
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='',
        description='SDF world file to load'
    )

    # Start Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v4 ', world],
        }.items(),
    )

    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'sekirei',
            '-topic', 'robot_description',
            '-z', '0.0',
        ],
        output='screen',
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # ROS2 Control Spawner - Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # ROS2 Control Spawner - Arm Controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['sekirei_arm_controller', '--controller-manager', '/controller_manager'],
    )

    # Delay arm controller after joint state broadcaster
    delay_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # MoveIt Nodes
    # Load configuration files
    moveit_controllers = load_yaml('sekirei_moveit_config', 'config/moveit_controllers.yaml')
    ompl_planning_config = load_yaml('sekirei_moveit_config', 'config/ompl_planning.yaml')
    joint_limits = load_yaml('sekirei_moveit_config', 'config/joint_limits.yaml')

    # Planning pipeline configuration
    ompl_planning_pipeline_config = {'move_group': ompl_planning_config}

    # Controller manager configuration
    moveit_controller_config = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': moveit_controllers
    }

    # Trajectory execution configuration
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # Planning scene monitor
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Joint limits for planning
    robot_description_planning = {'robot_description_planning': joint_limits}

    # Move Group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controller_config,
            planning_scene_monitor_parameters,
            {'use_sim_time': True},
        ],
    )

    # RViz
    rviz_config = os.path.join(pkg_sekirei_moveit, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            {'use_sim_time': True},
        ],
    )

    # Static TF for world to base_link
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link'],
    )

    # Bridge for clock (Gazebo time to ROS time)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    nodes_to_start = [
        declare_world_cmd,
        gz_sim,
        spawn_entity,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        delay_arm_controller_spawner,
        move_group_node,
        rviz_node,
        static_tf,
        clock_bridge,
    ]

    return LaunchDescription(nodes_to_start)
