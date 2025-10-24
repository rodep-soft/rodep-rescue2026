#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
import sys


class SekireiArmMover(Node):
    def __init__(self):
        super().__init__('sekirei_arm_mover')
        self._action_client = ActionClient(self, MoveGroup, '/move_action')

    def move_to_joint_positions(self, joint_values):
        """Move arm to specific joint positions
        joint_values: list of 6 floats for [joint1, joint2, joint3, joint4, joint5, joint6]
        """
        self.get_logger().info(f'Moving to joint positions: {joint_values}')

        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return False

        # Create goal message
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'sekirei_arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0

        # Set joint constraints
        joint_names = ['arm_joint1', 'arm_joint2', 'arm_joint3',
                      'arm_joint4', 'arm_joint5', 'arm_joint6']

        for name, value in zip(joint_names, joint_values):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = value
            constraint.tolerance_above = 0.001
            constraint.tolerance_below = 0.001
            constraint.weight = 1.0
            goal_msg.request.goal_constraints.append(
                Constraints(joint_constraints=[constraint])
            )

        # Send goal
        self.get_logger().info('Sending goal...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)

        return True


def main(args=None):
    rclpy.init(args=args)
    node = SekireiArmMover()

    # Example: Move to home position (all joints at 0)
    home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Example: Move to a custom position
    custom_position = [0.5, -0.5, 1.0, -0.3, 0.0, 0.0]

    print("\nSekirei Arm Control")
    print("=" * 50)
    print("Commands:")
    print("  1 - Move to home position (all zeros)")
    print("  2 - Move to custom position")
    print("  q - Quit")
    print("=" * 50)

    while rclpy.ok():
        cmd = input("\nEnter command: ").strip()

        if cmd == '1':
            node.move_to_joint_positions(home_position)
        elif cmd == '2':
            node.move_to_joint_positions(custom_position)
        elif cmd == 'q':
            break
        else:
            print("Invalid command!")

        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
