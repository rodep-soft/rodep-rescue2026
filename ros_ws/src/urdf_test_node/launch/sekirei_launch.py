import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = "urdf_test_node"

    pkg_share_dir = get_package_share_directory(pkg_name)

    # Path to sekirei.urdf file
    urdf_path = os.path.join(pkg_share_dir, "urdf", "sekirei.urdf")

    # Read URDF file content
    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # robot_state_publisher node
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": False}
        ],
    )

    # joint_state_publisher node (publishes default joint states)
    jsp_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": False}
        ],
    )

    # RViz2 node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg_share_dir, "rviz", "config.rviz")] if os.path.exists(os.path.join(pkg_share_dir, "rviz", "config.rviz")) else [],
    )

    return LaunchDescription(
        [
            rsp_node,
            jsp_node,
            rviz_node,
        ]
    )
