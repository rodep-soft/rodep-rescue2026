import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    moveit_config_pkg = get_package_share_directory('sekirei_moveit_config')
    urdf_pkg = get_package_share_directory('urdf_test_node')

    # Load URDF
    urdf_file = os.path.join(urdf_pkg, 'urdf', 'sekirei_moveit.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Static TF for world->base_link
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}],
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
        static_tf_node,
        robot_state_publisher,
        joy_node,
        joy_teleop_node,
    ])
