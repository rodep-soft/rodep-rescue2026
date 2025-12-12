from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 実機 or モックを切り替えるフラグ
    use_real_hw = LaunchConfiguration('use_real_hw')

    # xacro ファイル（さっき直した y_sekirei_moveit.xacro）
    robot_model_share = get_package_share_directory('robot_model')
    xacro_file = PathJoinSubstitution(
        [robot_model_share, 'urdf', 'y_sekirei_moveit.xacro']
    )

    robot_description = {
    'robot_description': ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_real_hw:=', use_real_hw
        ]),
        value_type=str
    )
}


    # さっき見せてくれた controller_manager の YAML
    sekirei_config_share = get_package_share_directory('sekirei_moveit_config')
    controllers_yaml = PathJoinSubstitution(
        [sekirei_config_share, 'config', 'ros2_controllers.yaml']  # ←このファイル名にしておくとわかりやすい
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='screen'
    )

    # joint_state_broadcaster
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # YAML の controller_manager 内で定義した名前に合わせる
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['sekirei_arm_controller'],
        output='screen'
    )

    declare_use_real_hw = DeclareLaunchArgument(
        'use_real_hw',
        default_value='false',
        description='Use real Dynamixel hardware (true=実機, false=モック)'
    )

    return LaunchDescription([
        declare_use_real_hw,
        ros2_control_node,
        jsb_spawner,
        arm_spawner,
    ])
