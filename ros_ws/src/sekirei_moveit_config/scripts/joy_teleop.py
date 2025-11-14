#!/usr/bin/env python3
"""Joy teleop for arm control - supports both /joint_commands fallback and
FollowJointTrajectory action mode for ros2_control controllers.

Use parameter `use_action` (bool, default true) to select action-mode.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')
        # Subscribe to joystick
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Publisher for joint commands to /joint_commands (fallback for fake controller)
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)
        # Also publish JointState to /joint_states so robot_state_publisher receives updates
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Current joint positions
        self.joint_positions = [0.0] * 6
        self.joint_names = [
            'arm_joint1', 'arm_joint2', 'arm_joint3',
            'arm_joint4', 'arm_joint5', 'arm_joint6'
        ]

        # Control parameters
        self.speed = 0.02  # radians per update
        self.deadzone = 0.1

        # Action-mode parameter
        self.declare_parameter('use_action', True)
        self.use_action = self.get_parameter('use_action').value

        # Action client (optional)
        self._action_client = None
        self._current_goal_future = None
        if self.use_action:
            self._action_client = ActionClient(self, FollowJointTrajectory,
                                               '/joint_trajectory_controller/follow_joint_trajectory')
            # send small trajectory goals at this interval
            self._send_interval = 0.1
            self._send_timer = self.create_timer(self._send_interval, self._send_action_goal)

        # Timer for publishing joint states (for visualization / fallback consumers)
        self.timer = self.create_timer(0.05, self.publish_joint_states)

        self.get_logger().info('Joy teleop started! use_action=%s' % str(self.use_action))

        self.get_logger().info('Left stick X: Joint 1 (base rotation)')
        self.get_logger().info('Left stick Y: Joint 2 (shoulder)')
        self.get_logger().info('Right stick Y: Joint 3 (elbow)')
        self.get_logger().info('Right stick X: Joint 4 (wrist1)')
        self.get_logger().info('L1/R1: Joint 5 (wrist2)')
        self.get_logger().info('L2/R2: Joint 6 (wrist3)')

    def apply_deadzone(self, value):
        """Apply deadzone to joystick value"""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        # publish both to /joint_commands (for compatibility) and /joint_states (for robot_state_publisher)
        try:
            self.joint_pub.publish(msg)
        except Exception:
            pass
        try:
            self.joint_state_pub.publish(msg)
        except Exception:
            pass

    def _send_action_goal(self):
        """Send a FollowJointTrajectory goal with the current joint positions.

        Best-effort: fire-and-forget goals at a low rate to provide continuous
        joystick control to a joint_trajectory_controller.
        """
        if not self.use_action or self._action_client is None:
            return

        # ensure server ready
        if not self._action_client.server_is_ready():
            return

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = list(self.joint_positions)
        # reach target in 0.5s
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = int(5e8)
        traj.points = [pt]

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        try:
            # send goal (don't block) - store future for reference
            send_fut = self._action_client.send_goal_async(goal_msg)
            self._current_goal_future = send_fut
        except Exception as e:
            self.get_logger().warn(f'Failed to send action goal: {e}')

    def joy_callback(self, msg: Joy):
        """Update joint positions based on joystick input"""
        if not msg.axes or len(msg.axes) < 4:
            return

        # Left stick X: Joint 1 (base rotation)
        delta1 = self.apply_deadzone(msg.axes[0]) * self.speed
        self.joint_positions[0] += delta1

        # Left stick Y: Joint 2 (shoulder)
        delta2 = self.apply_deadzone(msg.axes[1]) * self.speed
        self.joint_positions[1] += delta2


        # Right stick Y: Joint 3 (elbow)
        if len(msg.axes) > 4:
            delta3 = self.apply_deadzone(msg.axes[4]) * self.speed
            self.joint_positions[2] += delta3

        # Right stick X: Joint 4 (wrist1)
        if len(msg.axes) > 3:
            delta4 = self.apply_deadzone(msg.axes[3]) * self.speed
            self.joint_positions[3] += delta4

        # Buttons: L1/R1 for Joint 5 (wrist2)
        if len(msg.buttons) > 5:
            if msg.buttons[4]:  # L1
                self.joint_positions[4] -= self.speed
            elif msg.buttons[5]:  # R1
                self.joint_positions[4] += self.speed

        # L2/R2 for Joint 6 (wrist3)
        if len(msg.buttons) > 7:
            if msg.buttons[6]:  # L2
                self.joint_positions[5] -= self.speed
            elif msg.buttons[7]:  # R2
                self.joint_positions[5] += self.speed

        # Clamp positions to reasonable limits
        for i in range(6):
            self.joint_positions[i] = max(-math.pi, min(math.pi, self.joint_positions[i]))


def main(args=None):
    rclpy.init(args=args)

    joy_teleop = JoyTeleop()


    try:
        rclpy.spin(joy_teleop)
    except KeyboardInterrupt:
        pass

    joy_teleop.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        # rclpy may already be shutdown if a SIGINT was handled elsewhere; ignore
        pass



if __name__ == '__main__':
    main()
