from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # パッケージディレクトリ
    pkg_velodyne = get_package_share_directory('velodyne')
    pkg_slam = get_package_share_directory('slam_toolbox')

    # 1. Velodyne driver launch
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_velodyne, 'launch', 'velodyne-all-nodes-VLP16-launch.py')
        )
    )

    # # 2. PointCloud2 -> LaserScan
    # pointcloud_to_laserscan = Node(
    #     package='pointcloud_to_laserscan',
    #     executable='pointcloud_to_laserscan_node',
    #     name='velodyne_to_scan',
    #     parameters=[{
    #         'target_frame': 'base_link',
    #         'transform_tolerance': 0.01,
    #         'min_height': -0.1,
    #         'max_height': 0.1,
    #         'angle_min': -3.14159,
    #         'angle_max': 3.14159,
    #         'scan_time': 0.1,
    #         'range_min': 0.1,
    #         'range_max': 100.0
    #     }],
    #     remappings=[
    #         ('/cloud_in', '/velodyne_points'),
    #         ('/scan', '/scan')
    #     ],
    #     arguments=['--qos-reliability', 'reliable']  # ← これで RELIABLE に固定
    # )

    # 3. Static TFs
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

    # # 4. SLAM Toolbox (scan-only, async)
    # slam_node = Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox',
    #     parameters=[{
    #         'use_sim_time': False,
    #         'odom_frame': 'odom',
    #         'map_frame': 'map',
    #         'base_frame': 'base_link',
    #         'provide_odom_frame': True,
    #         'use_scan_matching': True
    #     }],
    #     remappings=[
    #         ('/scan', '/scan')
    #     ]
    # )

    # Simple slam launch
    # slam_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
    #     )
    # )

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_sync_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'odom_frame': 'odom', # 空だからodomなし
            'map_frame': 'map',
            'base_frame': 'base_link',
            'provide_odom_frame': 'true',  # ← odom 無しでも動く
            'use_scan_matching': 'true'
        }.items()
    )


    # 5. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_slam, 'rviz', 'slam_toolbox.rviz')]
    )

    return LaunchDescription([
        velodyne_launch,
        # pointcloud_to_laserscan,
        map_to_odom_tf,
        odom_to_base_tf,
        baselink_to_velodyne_tf,
        slam_node,
        rviz_node
    ])
