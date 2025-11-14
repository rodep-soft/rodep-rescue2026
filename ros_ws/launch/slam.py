from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Velodyne driver & transform nodes
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('velodyne'),
                'launch',
                'velodyne-all-nodes-VLP16-launch.py'
            )
        )
    )

    # SLAM Toolbox (Lidar-only / Odometryなしモード)
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        parameters=[{
            'use_sim_time': False,
            'queue_size': 50,
            'odom_frame': 'odom',            # ここは静的TFで接続
            'map_frame': 'map',
            'base_frame': 'base_link',
            'provide_odom_frame': True,      # map->odomのTFを出す
            'use_scan_matching': True,       # Lidar点群のみでSLAM
        }]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('slam_toolbox'),
            'rviz',
            'slam_toolbox.rviz'
        )]
    )

    # Static TFs
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','map','odom']
    )

    odom_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','odom','base_link']
    )

    baselink_to_velodyne_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','base_link','velodyne']
    )

    return LaunchDescription([
        velodyne_launch,
        map_to_odom_tf,
        odom_to_base_tf,
        baselink_to_velodyne_tf,
        slam_node,
        rviz_node
    ])
