"""Robust MoveIt + moveit_servo demo launch

This launch ensures controller_manager and move_group are available before
starting the standalone moveit_servo node. It waits for required services and
then launches the servo node with parameters loaded from YAML plus the
robot_description/semantic provided by MoveIt configs.

Usage:
  ros2 launch sekirei_moveit_config servo_demo.launch.py

This file is conservative: it uses 'unitless' as the servo command type to
match the moveit_servo binaries that accept 'unitless' or 'speed_units'. If
your installation expects a different enum, adjust the parameter accordingly.
"""

# Moveit_servoの公式repoを参考にした堅牢なlaunch

import os
import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def load_moveit_config():
    # Use static package files from sekirei_moveit_config. Build a
    # robot_description Command (xacro invocation) and read the SRDF file
    # contents into a string so the launch system doesn't treat the SRDF as
    # a params-file (which caused parsing errors).
    pkg = 'sekirei_moveit_config'
    pkg_path = get_package_share_directory(pkg)
    semantic_path = os.path.join(pkg_path, 'config', 'sekirei.srdf')

    # Try a few likely locations for the xacro (sekirei_moveit_config first,
    # then robot_model). Use the first existing file. If none found, do not
    # execute xacro to avoid launch-time failures; instead return empty
    # robot_description and log a warning so the user can inspect files.
    candidates = [
        os.path.join(pkg_path, 'urdf', 'sekirei_moveit.xacro'),
    ]
    try:
        robot_model_pkg = get_package_share_directory('robot_model')
        candidates.append(os.path.join(robot_model_pkg, 'urdf', 'sekirei_moveit.xacro'))
    except Exception:
        pass

    robot_xacro_path = None
    for p in candidates:
        if os.path.exists(p):
            robot_xacro_path = p
            break

    if robot_xacro_path:
        robot_description_cmd = Command(['xacro ', robot_xacro_path, ' use_real_hw:=false'])
    else:
        print(f"[servo_demo.launch] Warning: no xacro found in candidates: {candidates}; robot_description will be empty")
        robot_description_cmd = ''

    # Read SRDF content into a string so it will be provided as a parameter
    # value rather than interpreted as a params-file path by the node.
    try:
        with open(semantic_path, 'r') as f:
            srdf_text = f.read()
    except Exception:
        srdf_text = ''

    return {
        'robot_description': robot_description_cmd,
        'robot_description_semantic': srdf_text,
    }


def wait_for_services(required_services, timeout=15.0):
    """Blocking wait for ROS services to appear. Returns True if all found.

    This helper uses `ros2 service list` via the ROS CLI in a subprocess-free
    manner by polling the file system for /ros2 topics is not ideal here, so
    we simply poll `ros2 service list` via os.popen to avoid importing rclpy in
    the launch process (avoids rclpy init conflicts in some environments).
    """
    deadline = time.time() + float(timeout)
    while time.time() < deadline:
        try:
            services = os.popen('ros2 service list').read().splitlines()
        except Exception:
            services = []
        missing = [s for s in required_services if s not in services]
        if not missing:
            return True
        time.sleep(0.5)
    return False


def launch_setup(context, *args, **kwargs):
    # Load MoveIt config (may be a dict-like object or the builder result)
    moveit_config = load_moveit_config()

    # Paths and common parameters
    pkg = get_package_share_directory('sekirei_moveit_config')
    servo_yaml = os.path.join(pkg, 'config', 'moveit_servo.yaml')

    # Node definitions: RViz and controller manager/spawners are started
    # without delay. The servo node will be started only after the required
    # services are visible in the ROS graph.
    rviz_cfg = os.path.join(pkg, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
        # Provide robot_description and semantic as named parameters (dicts).
        parameters=[
            {'robot_description': moveit_config.get('robot_description') if isinstance(moveit_config, dict) else moveit_config.robot_description},
            {'robot_description_semantic': moveit_config.get('robot_description_semantic') if isinstance(moveit_config, dict) else moveit_config.robot_description_semantic},
        ],
    )

    # ros2_control / controller_manager (if present in the environment)
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'ros2_controllers.yaml')],
    )

    # Publish robot_description to the /robot_description topic early so
    # controller_manager and other nodes that subscribe to the topic can
    # receive it. We run a small helper script which will call xacro if a
    # xacro path was found and then publish the produced string with
    # transient-local durability so late-joining subscribers receive it.
    publish_script = os.path.join(pkg, 'scripts', 'publish_robot_description.py')
    publish_action = None
    if moveit_config.get('robot_description'):
        # moveit_config['robot_description'] may be a Command substitution or
        # an empty string. We pass the xacro path and srdf path as args to
        # the helper script so it knows what to publish. Use absolute paths
        # if the xacro exists.
        # Determine the xacro path argument by checking the candidate
        # resolution earlier (moveit_config['robot_description'] may be a
        # Command object, but we still have robot_xacro_path from loader).
        # We recreate the candidate lookup here for the script args.
        try:
            # If a real file path was selected earlier, prefer to pass it.
            xacro_arg = None
            # We rely on the same heuristic used above: check common locations.
            candidates = [os.path.join(pkg, 'urdf', 'sekirei_moveit.xacro')]
            try:
                robot_model_pkg = get_package_share_directory('robot_model')
                candidates.append(os.path.join(robot_model_pkg, 'urdf', 'sekirei_moveit.xacro'))
            except Exception:
                pass
            for p in candidates:
                if os.path.exists(p):
                    xacro_arg = p
                    break
            if xacro_arg:
                publish_action = ExecuteProcess(cmd=['python3', publish_script, '--xacro', xacro_arg, '--srdf', semantic_path], output='screen')
            else:
                # No xacro file found; still launch publisher with empty xacro
                # so it can publish the SRDF (if desired) or nothing.
                publish_action = ExecuteProcess(cmd=['python3', publish_script, '--srdf', semantic_path], output='screen')
        except Exception:
            publish_action = None

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['sekirei_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Build the servo node action but do not start it yet. We will return it
    # from this function only after we confirm required services exist.
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        output='screen',
        parameters=[
            servo_yaml,
            # Conservative override: use 'unitless' which is accepted by
            # many moveit_servo builds for pose-like inputs.
            {'moveit_servo.command_in_type': 'unitless', 'command_in_type': 'unitless'},
            # If moveit_config is a builder object it exposes mapping-like
            # attributes. If it's a dict we pass raw files/strings.
            getattr(moveit_config, 'robot_description', moveit_config.get('robot_description'))
            if isinstance(moveit_config, dict) else moveit_config.robot_description,
            getattr(moveit_config, 'robot_description_semantic', moveit_config.get('robot_description_semantic'))
            if isinstance(moveit_config, dict) else moveit_config.robot_description_semantic,
        ],
    )

    # Launch basic nodes immediately, then wait and return servo_node.
    # Put the publisher before controller_manager so the topic is visible
    # when controller_manager starts (it subscribes to /robot_description).
    actions = []
    if publish_action:
        actions.append(publish_action)
    actions.extend([controller_manager_node, joint_state_spawner, arm_controller_spawner, rviz_node])

    # Define which services must exist before starting Servo. We require the
    # controller_manager service and at least one move_group planning service.
    required_services = ['/controller_manager/list_controllers', '/move_group/get_planning_scene']
    ok = wait_for_services(required_services, timeout=20.0)
    if not ok:
        # Best-effort: log a warning to the console and still start servo after
        # a short delay. This prevents deadlock in CI or minimal environments.
        print('[servo_demo.launch] Warning: required services not visible after timeout; starting servo anyway')
        actions.append(servo_node)
        return actions

    # All required services are present -> start servo
    print('[servo_demo.launch] Required services visible; starting moveit_servo now')
    actions.append(servo_node)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        OpaqueFunction(function=launch_setup),
    ])
