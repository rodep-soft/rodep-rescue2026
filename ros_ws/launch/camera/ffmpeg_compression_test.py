from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # USB camera node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node',
        output='screen',
        parameters=[
            {
                'video_device': '/dev/video0',
                'image_width': 1280,
                'image_height': 720,
                'pixel_format': 'yuyv',
                'camera_frame_id': 'camera_frame',
            },
            {
                # ffmpeg_image_transport parameters
                'ffmpeg_image_transport.encoding': 'libx264',
                'ffmpeg_image_transport.profile': 'main',
                'ffmpeg_image_transport.preset': 'll',
                'ffmpeg_image_transport.gop_size': 15,
                'ffmpeg_image_transport.measure_performance': True,
            },
        ],
    )

    return LaunchDescription([
        usb_cam_node,
    ])
