import os
import yaml
import xacro

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _generate_urdf_from_xacro_real() -> str:
    xacro_path = os.path.join(
        get_package_share_directory("robot_model"),
        "urdf",
        "sekirei_moveit.xacro",
    )
    doc = xacro.process_file(xacro_path, mappings={"use_real_hw": "true"})
    return doc.toxml()


def _load_text(package: str, relative: str) -> str:
    path = os.path.join(get_package_share_directory(package), relative)
    with open(path, "r") as f:
        return f.read()


def _load_yaml(package: str, relative: str):
    path = os.path.join(get_package_share_directory(package), relative)
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _setup(context, *args, **kwargs):
    # === 1. MoveIt基本ファイル ===
    urdf_text = _generate_urdf_from_xacro_real()
    srdf_text = _load_text("sekirei_moveit_config", "config/sekirei.srdf")
    kin_yaml = _load_yaml("sekirei_moveit_config", "config/kinematics.yaml")
    ompl_yaml = _load_yaml("sekirei_moveit_config", "config/ompl_planning.yaml")
    cpp_yaml = _load_yaml("sekirei_moveit_config", "config/moveit_cpp.yaml")
    ctrl_yaml = _load_yaml("sekirei_moveit_config", "config/moveit_controllers.yaml")
    joints_limits_yaml = _load_yaml("sekirei_moveit_config", "config/joint_limits.yaml")

    # Servo param file path（←ここが重要：ロードしない）
    servo_params_file = os.path.join(
        get_package_share_directory("sekirei_moveit_config"),
        "config",
        "servo_parameters.yaml",
    )

    # === 2. MoveGroup params ===
    move_group_params = {
        "robot_description": urdf_text,
        "robot_description_semantic": srdf_text,
        "robot_description_kinematics": kin_yaml,
        "robot_description_planning": joints_limits_yaml,

        "use_sim_time": False,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,

        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "moveit_fake_controller_manager.fake_execution_type": "interpolate",
    }

    # === 3. MoveItCpp設定 ===
    if cpp_yaml:
        if (
            "planning_pipelines" in cpp_yaml
            and isinstance(cpp_yaml["planning_pipelines"].get("pipeline_names"), list)
        ):
            cpp_yaml["planning_pipelines"]["pipeline_names"] = ParameterValue(
                cpp_yaml["planning_pipelines"]["pipeline_names"]
            )
        if "ompl" in cpp_yaml and isinstance(cpp_yaml["ompl"].get("planning_plugins"), list):
            cpp_yaml["ompl"]["planning_plugins"] = ParameterValue(
                cpp_yaml["ompl"]["planning_plugins"]
            )
        move_group_params.update(cpp_yaml)

    # === 4. OMPL設定 ===
    if ompl_yaml:
        if isinstance(ompl_yaml.get("planning_plugins"), list):
            ompl_yaml["planning_plugins"] = ParameterValue(ompl_yaml["planning_plugins"])
        if (
            "sekirei_arm" in ompl_yaml
            and isinstance(ompl_yaml["sekirei_arm"].get("planner_configs"), list)
        ):
            ompl_yaml["sekirei_arm"]["planner_configs"] = ParameterValue(
                ompl_yaml["sekirei_arm"]["planner_configs"]
            )
        for k, v in ompl_yaml.items():
            move_group_params[f"ompl.{k}"] = v

    # === 5. controller設定 ===
    if ctrl_yaml and "moveit_simple_controller_manager" in ctrl_yaml:
        scm = ctrl_yaml["moveit_simple_controller_manager"]
        if isinstance(scm.get("controller_names"), list):
            scm["controller_names"] = ParameterValue(scm["controller_names"])
        move_group_params.update(ctrl_yaml)

    nodes = []

    # === 6. static TF world->base_link ===
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_world_base",
            output="log",
            arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        )
    )

    # === 7. move_group ===
    nodes.append(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[move_group_params],
        )
    )

    # === 7.5 MoveIt Servo ===
    nodes.append(
        Node(
            package="moveit_servo",
            executable="servo_node",   # Jazzyではこれ（あなたのログで確認済み）
            name="servo_node",
            output="screen",
            parameters=[
                # Servoノードにもrobot modelを渡す（別プロセスなので必要）
                {
                    "robot_description": urdf_text,
                    "robot_description_semantic": srdf_text,
                    "robot_description_kinematics": kin_yaml,
                    "use_sim_time": False,
                },
                servo_params_file,  # ←ここが重要：YAMLはファイルで渡す
            ],
        )
    )

    # === 8. RViz2 ===
    rviz_cfg = os.path.join(
        get_package_share_directory("sekirei_moveit_config"),
        "config",
        "moveit.rviz",
    )
    rviz_params = {
        "robot_description": urdf_text,
        "robot_description_semantic": srdf_text,
        "robot_description_kinematics": kin_yaml,
    }
    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_cfg] if os.path.exists(rviz_cfg) else [],
            parameters=[rviz_params],
        )
    )

    # === 9. joy_node ===
    nodes.append(
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{
                "device_id": 0,
                "deadzone": 0.1,
            }],
        )
    )

    # === 10. Joy teleop（Servo版）===
    # ここは「あなたがビルドした実行ファイル名」に合わせる
    nodes.append(
        Node(
            package="sekirei_moveit_config",
            executable="joy_teleop",   # ←ここがServo teleopの実行ファイル名
            name="joy_teleop",
            output="screen",
            parameters=[{
                "servo_node_name": "/servo_node",
                "planning_frame": "base_link",  # servo_parameters.yamlと合わせる
                "base_vel_topic": "/base_velocity_controller/commands",
                "jog_joint4_name": "arm_joint4",
                "jog_joint6_name": "arm_joint6",
                "stick_deadzone": 0.20,
                "trigger_deadzone": 0.05,
                "publish_hz": 50.0,
                "axis_dpad_y": 7,
                "axis_dpad_x": 6,
            }],
        )
    )

    return nodes


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_setup)])