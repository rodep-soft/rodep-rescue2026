from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # USB camera node - 低解像度・低フレームレートで軽量化
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node',
        output='screen',
        parameters=[
            {
                'video_device': '/dev/video0',
                'image_width': 640,  # バランス: 640x480
                'image_height': 480,
                'pixel_format': 'yuyv',  # YUVフォーマット
                'camera_frame_id': 'camera_frame',
                'framerate': 30.0,  # 15fps
            },
        ],
    )

    # Raw -> JPEG compressed (シンプルな変換)
    republish_node = Node(
        package='image_transport',
        executable='republish',
        name='image_republisher',
        arguments=[
            'raw',
            'compressed',
            '--ros-args',
            '--remap', 'in:=/image_raw',
            '--remap', 'out/compressed:=/image_raw/compressed',
        ],
        parameters=[
            {
                'compressed.jpeg_quality': 50,  # バランス品質
            }
        ],
        output='screen',
    )

    return LaunchDescription([
        usb_cam_node,
        republish_node,
    ])
