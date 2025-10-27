from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="image_tools",
                executable="cam2image",
                # name='main_camera',
                parameters=[
                    {
                        # 'frequency': 30,
                        "device_id": 2,
                        # 'burger_mode': 'true',
                    }
                ],
            ),
            # Node(
            #     package='image_tools',
            #     executable='cam2image',
            #     name='second_camera',
            #     parameters=[{
            #         'frequency': 30,
            #         'device_id': 1,
            #     }]
            # ),
            Node(
                package="image_transport",
                executable="republish",
                # name='image_republisher',
                arguments=[
                    "raw",
                    "compressed",  # 入力/出力 transport
                    "--ros-args",
                    "--remap",
                    "in:=/image",  # 入力トピック remap
                    "--remap",
                    "out/compressed:=/image/compressed",  # 出力トピック remap
                ],
                output="screen",
            ),
            # Node(
            #     package='image_transport',
            #     executable='republish',
            #     name='image_republisher',
            #     parameters=[{
            #         'in': '/camera/image_raw1',
            #         'out': '/camera/image_republished1/compressed'
            #     }]
            # ),
            # Node(
            #     package='image_tools',
            #     executable='showimage',
            #     name='image_viewer',
            #     parameters=[{
            #         'image_topic': '/camera/image_republished0/compressed'
            #     }]
            # ),
            # Node(
            #     package='image_tools',
            #     executable='showimage',
            #     name='image_viewer',
            #     parameters=[{
            #         'image_topic': '/camera/image_republished1/compressed'
            #     }]
            # )
        ]
    )
