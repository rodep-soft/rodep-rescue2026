# メインのアーム実機制御、シミュレーション用ファイル

import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers.on_process_start import OnProcessStart
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch.substitutions import Command
# MoveItConfigsBuilder is optional (may not be installed in minimal environments)
try:
    from moveit_configs_utils import MoveItConfigsBuilder
except Exception:
    MoveItConfigsBuilder = None
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
    urdf_pkg = get_package_share_directory('robot_model')

    # Load URDF (use MoveIt-compatible version with collision and inertial)
    # Check if Dynamixel hardware is connected by looking for USB device
    import glob
    dynamixel_connected = False
    usb_devices = glob.glob('/dev/ttyUSB*')
    if usb_devices:
        print(f"[demo.launch.py] Found USB devices: {usb_devices}")
        # Check if dynamixel_hardware_interface package is available
        try:
            _ = get_package_share_directory('dynamixel_hardware_interface')
            dynamixel_connected = True
            print("[demo.launch.py] Dynamixel hardware detected - using sekirei_moveit.urdf with dynamixel_hardware_interface")
        except PackageNotFoundError:
            print("[demo.launch.py] USB device found but dynamixel_hardware_interface not installed")

    # if dynamixel_connected:
    #     urdf_file = os.path.join(moveit_config_pkg, 'urdf', 'sekirei_moveit.urdf')
    # else:
    #     urdf_file = os.path.join(urdf_pkg, 'urdf', 'sekirei_moveit_dummy.urdf')
    #     print("[demo.launch.py] No Dynamixel hardware detected - using sekirei_moveit_dummy.urdf with mock_components")

    # スペースは必要な為絶対に消さないこと。消すとうごかない
    urdf_xacro_path = os.path.join(urdf_pkg, 'urdf', 'sekirei_moveit.xacro')
    robot_description = Command([
        'xacro ',
        urdf_xacro_path,
        ' use_real_hw:=', LaunchConfiguration('use_real_hw')
    ])

    # Load SRDF
    srdf_file = os.path.join(moveit_config_pkg, 'config', 'sekirei.srdf')
    print(">>> SRDF path:", srdf_file)
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    # Kinematics config
    kinematics_yaml = load_yaml('sekirei_moveit_config', 'config/kinematics.yaml')

    # Planning config
    ompl_planning_yaml = load_yaml('sekirei_moveit_config', 'config/ompl_planning.yaml')

    # Joint limits config
    joint_limits_yaml = load_yaml('sekirei_moveit_config', 'config/joint_limits.yaml')

    # MoveIt Cpp config (planning pipelines)
    moveit_cpp_yaml = load_yaml('sekirei_moveit_config', 'config/moveit_cpp.yaml')



    # Load moveit_controllers.yaml for ros2_control execution
    moveit_controllers = load_yaml('sekirei_moveit_config', 'config/moveit_controllers.yaml')

    # Trajectory execution and moveit_controller_manager parameters
    trajectory_execution = {
        'moveit_manage_controllers': False,  # Let controller_manager handle this
        'trajectory_execution.allowed_execution_duration_scaling': 3.0,  # Very lenient timing
        'trajectory_execution.allowed_goal_duration_margin': 5.0,  # Allow много time
        'trajectory_execution.allowed_start_tolerance': 0.5,  # Very lenient start position tolerance
        'trajectory_execution.execution_duration_monitoring': False,  # Disable strict monitoring
        'trajectory_execution.wait_for_trajectory_completion': True,  # Wait for completion
    }    # ros2_control parameters (instead of fake execution)
    ros2_controllers_path = os.path.join(moveit_config_pkg, 'config', 'ros2_controllers.yaml')
    ros2_control_params = load_yaml('sekirei_moveit_config', 'config/ros2_controllers.yaml')

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

        # Convert response_adapters list to ParameterValue
        if 'response_adapters' in ompl_planning_yaml and isinstance(ompl_planning_yaml['response_adapters'], list):
            ompl_planning_yaml['response_adapters'] = ParameterValue(ompl_planning_yaml['response_adapters'])

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

    # Add joint limits configuration
    if joint_limits_yaml:
        move_group_params['robot_description_planning'] = joint_limits_yaml

    # Add trajectory execution parameters
    move_group_params.update(trajectory_execution)
    move_group_params.update(planning_scene_monitor_parameters)

    # Add controller configurations from moveit_controllers.yaml
    if moveit_controllers:
        # Convert controller_names list to ParameterValue if needed
        if 'moveit_simple_controller_manager' in moveit_controllers:
            scm = moveit_controllers['moveit_simple_controller_manager']
            if 'controller_names' in scm and isinstance(scm['controller_names'], list):
                scm['controller_names'] = ParameterValue(scm['controller_names'])
        move_group_params.update(moveit_controllers)

    # ros2_control node (controller_manager) with dummy hardware — only launch if available
    ros2_nodes = []
    try:
        # Will raise PackageNotFoundError if controller_manager isn't installed in the environment
        _ = get_package_share_directory('controller_manager')

    # We'll avoid TimerAction (it's flaky). Use OpaqueFunction-based
    # event handlers below that wait for controller_manager services
    # before starting spawners.

        # Load controller parameters from YAML
        controller_manager_params = ros2_control_params.get('controller_manager', {})
        controller_manager_params['robot_description'] = robot_description

        ros2_control_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_manager_params, ros2_controllers_path],
            output='screen',
        )

        # Spawners for controllers (delayed to wait for controller_manager)
        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        )

        sekirei_arm_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['sekirei_arm_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        )

        # joint_state_publisher_node = Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     output='screen',
        # )


        ros2_nodes.append(ros2_control_node)
        # Instead of TimerAction-based delays, register an event handler that
        # waits for controller_manager services and then launches the
        # spawners. This avoids the TimerAction bug and is more deterministic.
        def _start_spawners_when_ready(context, *a, **k):
            import time, os
            deadline = time.time() + 20.0
            required = ['/controller_manager/list_controllers']
            while time.time() < deadline:
                try:
                    services = os.popen('ros2 service list').read().splitlines()
                except Exception:
                    services = []
                if all(s in services for s in required):
                    # Return actions to start the spawners
                    return [joint_state_broadcaster_spawner, sekirei_arm_controller_spawner]
                time.sleep(0.5)
            # Timeout -> still start spawners to avoid deadlock in constrained envs
            print('[demo.launch] Warning: controller_manager services not visible; starting spawners anyway')
            return [joint_state_broadcaster_spawner, sekirei_arm_controller_spawner]

        ros2_nodes.append(RegisterEventHandler(
            OnProcessStart(target_action=ros2_control_node, on_start=[OpaqueFunction(function=_start_spawners_when_ready)])
        ))
    except PackageNotFoundError:
        print("[launch warning] controller_manager package not found; skipping ros2_control_node and spawners. Falling back to MoveIt simple controller manager (fake controllers). Install ros2_control/ros2_controllers to enable hardware controllers.")

    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        namespace='',
        parameters=[move_group_params],
        arguments=['--ros-args', '--log-level', 'INFO', '--log-level', 'moveit.trajectory_execution:=DEBUG'],
    )

    # RViz
    rviz_config_file = os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        namespace='', # バグるからかえない
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
            {'robot_description': robot_description},
            {'publish_robot_description': False},  # Don't republish, ros2_control_node already does
        ],
    )

    # Moveit Servo: don't rely on MoveItConfigsBuilder being present. We'll provide
    # the necessary parameters (robot_description and robot_description_semantic)
    # directly to the node and start it after move_group is launched so it can
    # find the semantic description without timing out.
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        namespace='', # バグるからかえない
        parameters=[
            # Load YAML and also explicitly set the command type under both
            # the moveit_servo.* prefixed name and the plain name so the
            # node definitely sees the setting at startup.
            {'moveit_servo.command_in_type': 'unitless', 'command_in_type': 'unitless'},
            # {'command_in_type': 'unitless'},
            os.path.join(moveit_config_pkg, 'config', 'moveit_servo.yaml'),
            # Ensure the servo starts in a command mode the installed
            # moveit_servo supports. The binary on this system accepts
            # 'unitless' or 'speed_units' — use 'unitless' for pose-like
            # external pose target commands.
            # {'moveit_servo.command_in_type': 'unitless', 'command_in_type': 'unitless'},
            # Ensure servo_node sees SRDF/URDF/kinematics on startup
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic},
            {'robot_description_kinematics': kinematics_yaml},
        ],
    )

    # Don't use joint_state_publisher - controller_manager will handle joint states via ros2_control

    # Do not start servo_node immediately: move_group should be up and the
    # robot description semantic should be available first. Start core nodes now.
    nodes_to_start = [
        static_tf_node,
        robot_state_publisher,
    ]

    # add ros2_control nodes if available
    nodes_to_start += ros2_nodes

    # move_group and rviz are added after controller-related nodes
    nodes_to_start += [
        move_group_node,
        rviz_node,
    ]

    # Start servo_node after move_group process starts so it can obtain
    # SRDF/robot_description and avoid startup races (semantic description timeout).
    # Register an event handler to launch the servo only after move_group is up.
    # Start servo shortly after move_group starts. Wrap servo_node in a
    # TimerAction to give move_group a small grace period for internal
    # initialization (planning scene, parameter callbacks, etc.). This is a
    # pragmatic workaround for intermittent race conditions where servo sees
    # parameters/topics but still reports "Command type has not been set".

    # TimerAction は使わない（バグるので） -> OnProcessStart + OpaqueFunction
    # を使って move_group のサービス可視性を確認してから servo を起動する。
    def _start_servo_when_ready(context, *a, **k):
        import time, os
        deadline = time.time() + 20.0
        required = ['/move_group/get_planning_scene']
        while time.time() < deadline:
            try:
                services = os.popen('ros2 service list').read().splitlines()
            except Exception:
                services = []
            if all(s in services for s in required):
                return [servo_node]
            time.sleep(0.5)
        print('[demo.launch] Warning: move_group services not visible after timeout; starting servo anyway')
        return [servo_node]

    start_servo_event = RegisterEventHandler(
        OnProcessStart(target_action=move_group_node, on_start=[OpaqueFunction(function=_start_servo_when_ready)])
    )
    nodes_to_start.append(start_servo_event)

    # Add rosbridge_server for Flutter UI communication
    use_rosbridge = LaunchConfiguration('use_rosbridge')
    if use_rosbridge.perform(context) == 'true':
        rosbridge_node = Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
                'address': '0.0.0.0',  # Bind to all network interfaces (allow external access)
            }],
            output='screen',
        )
        nodes_to_start.append(rosbridge_node)

    return nodes_to_start


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rosbridge',
            default_value='false',
            description='Launch rosbridge_server for web/Flutter UI connection'
        ),
        DeclareLaunchArgument(
            'use_real_hw',
            default_value='false', # デフォルトではシミュレーション
            description='Use real hardware if connected'
        ),
        OpaqueFunction(function=launch_setup)
    ])
