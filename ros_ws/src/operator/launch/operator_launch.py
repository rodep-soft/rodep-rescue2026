from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
        Node(
            package="joy",
            # namespace="",
            executable="joy_node",
            name="pub_joy"
        ),
		Node(
			package='operator',
			executable='operator_node',
			name='operator_node',
			output='screen',
		),
	])
