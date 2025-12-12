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
        "y_sekirei_moveit.xacro",
    )
    # use_real_hw:=true を指定して展開
    doc = xacro.process_file(
        xacro_path,
        mappings={"use_real_hw": "true"}
    )
    return doc.toxml()



def _load_text(package: str, relative: str) -> str:
    """パッケージ内のテキストファイル(URDF, SRDFなど)を読むヘルパ."""
    path = os.path.join(get_package_share_directory(package), relative)
    with open(path, "r") as f:
        return f.read()


def _load_yaml(package: str, relative: str):
    """YAML設定ファイルを読むヘルパ."""
    path = os.path.join(get_package_share_directory(package), relative)
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _setup(context, *args, **kwargs):
    # === 1. 各種ファイル読み込み ===
    # ここは「今まで MoveIt で使っていた sekirei.urdf」を読む想定
    urdf_text = _generate_urdf_from_xacro_real()
    srdf_text = _load_text("sekirei_moveit_config", "config/sekirei.srdf")
    kin_yaml = _load_yaml("sekirei_moveit_config", "config/kinematics.yaml")
    ompl_yaml = _load_yaml("sekirei_moveit_config", "config/ompl_planning.yaml")
    cpp_yaml = _load_yaml("sekirei_moveit_config", "config/moveit_cpp.yaml")
    ctrl_yaml = _load_yaml("sekirei_moveit_config", "config/moveit_controllers.yaml")
    joints_limits_yaml = _load_yaml("sekirei_moveit_config", "config/joint_limits.yaml")

    # === 2. MoveGroup 用パラメータのベース ===
    move_group_params = {
        "robot_description": urdf_text,
        "robot_description_semantic": srdf_text,
        "robot_description_kinematics": kin_yaml,
        "robot_description_planning": joints_limits_yaml,

        # 時間・シーン周り
        "use_sim_time": False,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,

        # 実行周り（適宜調整可）
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "moveit_fake_controller_manager.fake_execution_type": "interpolate",
    }

    # === 3. MoveItCpp 設定（moveit_cpp.yaml） ===
    if cpp_yaml:
        # pipeline_names が list だと ParameterValue に包んだほうが安全
        if (
            "planning_pipelines" in cpp_yaml
            and isinstance(cpp_yaml["planning_pipelines"].get("pipeline_names"), list)
        ):
            cpp_yaml["planning_pipelines"]["pipeline_names"] = ParameterValue(
                cpp_yaml["planning_pipelines"]["pipeline_names"]
            )

        # ompl.planning_plugins も list の場合は包む
        if "ompl" in cpp_yaml and isinstance(
            cpp_yaml["ompl"].get("planning_plugins"), list
        ):
            cpp_yaml["ompl"]["planning_plugins"] = ParameterValue(
                cpp_yaml["ompl"]["planning_plugins"]
            )

        move_group_params.update(cpp_yaml)

    # === 4. OMPL 設定（ompl_planning.yaml） ===
    if ompl_yaml:
        # ルートに planning_plugins が list である場合
        if isinstance(ompl_yaml.get("planning_plugins"), list):
            ompl_yaml["planning_plugins"] = ParameterValue(
                ompl_yaml["planning_plugins"]
            )

        # sekirei_arm 配下の planner_configs が list なら包む
        if (
            "sekirei_arm" in ompl_yaml
            and isinstance(ompl_yaml["sekirei_arm"].get("planner_configs"), list)
        ):
            ompl_yaml["sekirei_arm"]["planner_configs"] = ParameterValue(
                ompl_yaml["sekirei_arm"]["planner_configs"]
            )

        # OMPL の設定は "ompl.xxx" というキーで move_group に渡す
        for k, v in ompl_yaml.items():
            move_group_params[f"ompl.{k}"] = v

    # === 5. MoveIt コントローラ設定（moveit_controllers.yaml） ===
    # ここで real の FollowJointTrajectory controller（sekirei_arm_controller）
    # を参照しているはず。
    if ctrl_yaml and "moveit_simple_controller_manager" in ctrl_yaml:
        scm = ctrl_yaml["moveit_simple_controller_manager"]
        if isinstance(scm.get("controller_names"), list):
            scm["controller_names"] = ParameterValue(scm["controller_names"])
        move_group_params.update(ctrl_yaml)

    nodes = []

    # === 6. world -> base_link の static TF ===
    # （bringup 側で同じものを出しているなら、ここは削ってもOK）
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_world_base",
            output="log",
            arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        )
    )

    # ★ここでは robot_state_publisher は起動しない
    #   → 実機 bringup (sekirei_robot_state_publisher.launch.py) 側で
    #      /robot_description + /tf を出している前提

    # === 7. MoveGroup ノード ===
    nodes.append(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[move_group_params],
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
    joy_node= Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone' : 0.1,
        }]
    )
    nodes.append(joy_node)

    joy_teleop= Node(
        package='robot_model',
        executable='joy_teleop',
        name='joy_teleop',
        output='screen',
        parameters=[{
            'robot_description': urdf_text,
            'robot_description_semantic': srdf_text,
            'robot_description_kinematics': kin_yaml,
        }]
    )
    nodes.append(joy_teleop)

    return nodes


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_setup)])
