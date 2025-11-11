import os
import yaml
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

    #ファイル読み込み関数
def _load_text(package: str, relative: str) -> str:
    path = os.path.join(get_package_share_directory(package), relative)
    with open(path, 'r') as f:
        return f.read()

def _load_yaml(package: str, relative: str):
    path = os.path.join(get_package_share_directory(package), relative)
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def _setup(context, *args, **kwargs):
    # 必要ファイルの読み込み
    urdf_text  = _load_text('urdf_test_node', 'urdf/sekirei_moveit.urdf')
    srdf_text  = _load_text('sekirei_moveit_config', 'config/sekirei.srdf')
    kin_yaml   = _load_yaml('sekirei_moveit_config', 'config/kinematics.yaml')
    ompl_yaml  = _load_yaml('sekirei_moveit_config', 'config/ompl_planning.yaml')
    cpp_yaml   = _load_yaml('sekirei_moveit_config', 'config/moveit_cpp.yaml')
    ctrl_yaml  = _load_yaml('sekirei_moveit_config', 'config/moveit_controllers.yaml')

    # MoveGroup用のパラメータを構築
    move_group_params = {
        'robot_description': urdf_text,
        'robot_description_semantic': srdf_text,
        'robot_description_kinematics': kin_yaml,
        # 便利フラグ
        'use_sim_time': False,
        # 監視関連
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        # 実行まわり
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'moveit_fake_controller_manager.fake_execution_type': 'interpolate',
    }

    # MoveIt Cpp 設定
    if cpp_yaml:
        if 'planning_pipelines' in cpp_yaml and isinstance(cpp_yaml['planning_pipelines'].get('pipeline_names'), list):
            cpp_yaml['planning_pipelines']['pipeline_names'] = ParameterValue(
                cpp_yaml['planning_pipelines']['pipeline_names'])
        if 'ompl' in cpp_yaml and isinstance(cpp_yaml['ompl'].get('planning_plugins'), list):
            cpp_yaml['ompl']['planning_plugins'] = ParameterValue(cpp_yaml['ompl']['planning_plugins'])
        move_group_params.update(cpp_yaml)

    # OMPL 設定
    if ompl_yaml:
        if isinstance(ompl_yaml.get('planning_plugins'), list):
            ompl_yaml['planning_plugins'] = ParameterValue(ompl_yaml['planning_plugins'])
        if 'sekirei_arm' in ompl_yaml and isinstance(ompl_yaml['sekirei_arm'].get('planner_configs'), list):
            ompl_yaml['sekirei_arm']['planner_configs'] = ParameterValue(ompl_yaml['sekirei_arm']['planner_configs'])
        for k, v in ompl_yaml.items():
            move_group_params[f'ompl.{k}'] = v

    # コントローラ設定
    if ctrl_yaml and 'moveit_simple_controller_manager' in ctrl_yaml:
        scm = ctrl_yaml['moveit_simple_controller_manager']
        if isinstance(scm.get('controller_names'), list):
            scm['controller_names'] = ParameterValue(scm['controller_names'])
        move_group_params.update(ctrl_yaml)

    # TF, RSP
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0','0','0','0','0','0','world','base_link']
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': urdf_text}]
    )

    # MoveGroup
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[move_group_params],
    )

    # RViz
    rviz_cfg = os.path.join(get_package_share_directory('sekirei_moveit_config'), 'config', 'moveit.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
        parameters=[{
            'robot_description': urdf_text,
            'robot_description_semantic': srdf_text,
            'robot_description_kinematics': kin_yaml,
        }]
    )
    #  Joyノード
    joy_node= Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    #  アームコントローラノード
    arm_controller = Node(
        package='urdf_test_node',
        executable='joy_teleop',
        name='joy_teleop',
        output='screen',
        parameters=[{
            'robot_description': urdf_text,
            'robot_description_semantic': srdf_text,
            'robot_description_kinematics': kin_yaml,
        }]
    )

    # フェイクコントローラ
    fake_ctrl = Node(
        package='sekirei_moveit_config',
        executable='fake_controller.py',
        name='fake_controller',
        output='screen',
    )

    return [static_tf, rsp, joy_node,move_group ,rviz, fake_ctrl, arm_controller]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_setup)])