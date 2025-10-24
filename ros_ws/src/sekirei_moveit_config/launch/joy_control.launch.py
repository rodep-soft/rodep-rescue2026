import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Include the main demo launch
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('sekirei_moveit_config'),
                'launch',
                'demo.launch.py'
            )
        ])
    )
    
    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }]
    )
    
    # Joy teleop (directly controls joints)
    joy_teleop_node = Node(
        package='sekirei_moveit_config',
        executable='joy_teleop.py',
        name='joy_teleop',
        output='screen',
    )
    
    return LaunchDescription([
        demo_launch,
        joy_node,
        joy_teleop_node,
    ])
