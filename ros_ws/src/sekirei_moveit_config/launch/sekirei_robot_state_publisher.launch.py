from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 実機 or モックを切り替えるフラグ
    use_real_hw = LaunchConfiguration('use_real_hw')

    # xacro ファイル
    robot_model_share = get_package_share_directory('robot_model')
    xacro_file = PathJoinSubstitution(
        [robot_model_share, 'urdf', 'y_sekirei_moveit.xacro']
    )

    # ★ここで ros2_control_node に渡す robot_description を生成
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ',          # 最後にスペースを入れるのがポイント
                xacro_file,
                ' use_real_hw:=',  # ここも前にスペース
                use_real_hw
            ]),
            value_type=str,
        )
    }

    # ros2_control の設定 YAML
    sekirei_config_share = get_package_share_directory('sekirei_moveit_config')
    controllers_yaml = PathJoinSubstitution(
        [sekirei_config_share, 'config', 'ros2_controllers.yaml']
    )

    # robot_state_publisher ノード
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # ros2_control_node 本体（ここに robot_description を渡す）
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

    # アーム用 joint_trajectory_controller
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['sekirei_arm_controller'],
        output='screen'
    )

    # 実機フラグ
    declare_use_real_hw = DeclareLaunchArgument(
        'use_real_hw',
        default_value='false',
        description='Use real Dynamixel hardware (true=実機, false=モック)'
    )

    return LaunchDescription([
        declare_use_real_hw,
        robot_state_publisher,
        ros2_control_node,
        jsb_spawner,
        arm_spawner,
    ])
