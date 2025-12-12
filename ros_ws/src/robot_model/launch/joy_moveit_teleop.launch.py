from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ジョイコンを /joy に出すノード
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'device_id': 0,   # 必要なら変える
                'deadzone': 0.1,
            }],
        ),

        # さっきの C++ ノード（JoyMoveItTeleop）
        Node(
            package='robot_model',   # ← CMakeLists.txt の PROJECT_NAME
            executable='joy_teleop', # ← add_executable(joy_teleop ...) の名前
            name='joy_moveit_teleop',
            output='screen',
        ),
    ])
