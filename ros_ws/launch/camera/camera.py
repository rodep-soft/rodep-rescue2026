from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_tools',
            executable='cam2image',
            name='main_camera',
            parameters=[{
                'frequency': 30,
                'device_id': 0,
            }]
        ),

        Node(
            package='image_tools',
            executable='cam2image',
            name='second_camera',
            parameters=[{
                'frequency': 30,
                'device_id': 1,
            }]
        )

    ])