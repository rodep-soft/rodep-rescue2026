#!/usr/bin/env python3
"""
MoveIt Bridge Node for Flutter UI

Subscribes to /move_to_pose topic and executes named poses via MoveIt.
This bridges the gap between simple Flutter UI commands and MoveIt actions.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from sensor_msgs.msg import JointState
import yaml


class MoveItBridge(Node):
    def __init__(self):
        super().__init__('moveit_bridge')

        # Subscribe to simple pose commands from Flutter
        self.pose_sub = self.create_subscription(
            String,
            '/move_to_pose',
            self.pose_callback,
            10
        )

        # Subscribe to joint states for feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Action client to MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')

        # Publishers for status
        self.status_pub = self.create_publisher(String, '/arm_status', 10)

        # Named poses definition (from sekirei.srdf)
        self.named_poses = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'ready': [0.0, 0.174533, 0.174533, 0.174533, 0.0, 0.0],
            'up': [0.0, -1.5708, 0.0, 0.0, 0.0, 0.0],
            'forward': [0.0, 0.0, 1.5708, -1.5708, 0.0, 0.0],
            'compact': [0.0, 1.5708, 1.5708, 1.5708, 0.0, 0.0],
            'left': [1.5708, 0.0, 0.0, 0.0, 0.0, 0.0],
            'right': [-1.5708, 0.0, 0.0, 0.0, 0.0, 0.0],
            'back': [3.14159, 0.0, 0.0, 0.0, 0.0, 0.0],
        }

        self.current_joint_state = None
        self.is_moving = False

        self.get_logger().info('MoveIt Bridge initialized')
        self.get_logger().info(f'Available poses: {list(self.named_poses.keys())}')

        # Wait for action server
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('Connected to MoveGroup action server')

    def joint_state_callback(self, msg):
        """Store current joint state for feedback"""
        self.current_joint_state = msg

    def pose_callback(self, msg):
        """Handle pose command from Flutter UI"""
        pose_name = msg.data.strip().lower()

        self.get_logger().info(f'Received pose command: {pose_name}')

        if pose_name not in self.named_poses:
            self.get_logger().error(f'Unknown pose: {pose_name}')
            self.publish_status(f'error: unknown pose "{pose_name}"')
            return

        if self.is_moving:
            self.get_logger().warn('Already moving, ignoring new command')
            return

        # Execute the pose
        self.move_to_pose(pose_name)

    def move_to_pose(self, pose_name):
        """Execute movement to named pose"""
        self.is_moving = True
        self.publish_status(f'moving to {pose_name}')

        joint_positions = self.named_poses[pose_name]

        # Create MoveGroup goal with joint constraints
        goal = MoveGroup.Goal()
        goal.request.group_name = 'sekirei_arm'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1

        # Set start state to current
        goal.request.start_state.is_diff = True

        # Create joint constraints for the goal
        constraints = Constraints()
        joint_names = [
            'arm_joint1', 'arm_joint2', 'arm_joint3',
            'arm_joint4', 'arm_joint5', 'arm_joint6'
        ]

        for i, (name, position) in enumerate(zip(joint_names, joint_positions)):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)

        goal.request.goal_constraints.append(constraints)

        # Set planning options
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True

        # Send goal
        self.get_logger().info(f'Sending goal to move to {pose_name}')
        send_goal_future = self.move_group_client.send_goal_async(goal)
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, pose_name)
        )

    def goal_response_callback(self, future, pose_name):
        """Handle goal acceptance"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected for pose {pose_name}')
            self.publish_status(f'error: goal rejected')
            self.is_moving = False
            return

        self.get_logger().info(f'Goal accepted for {pose_name}')

        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.result_callback(future, pose_name)
        )

    def result_callback(self, future, pose_name):
        """Handle execution result"""
        result = future.result().result

        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info(f'Successfully moved to {pose_name}')
            self.publish_status(f'success: reached {pose_name}')
        else:
            self.get_logger().error(
                f'Failed to move to {pose_name}. Error code: {result.error_code.val}'
            )
            self.publish_status(f'error: failed with code {result.error_code.val}')

        self.is_moving = False

    def publish_status(self, status):
        """Publish status for Flutter UI"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        bridge = MoveItBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
