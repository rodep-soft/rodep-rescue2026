#!/usr/bin/env python3
"""
Joy teleop for arm control - directly controls joint positions.
Allows real-time joystick control of the robot arm.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
import math


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')
        
        # Subscribe to joy
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Publish joint commands to /joint_commands (will be read by fake_controller or other nodes)
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            10
        )
        
        # Current joint positions
        self.joint_positions = [0.0] * 6
        self.joint_names = [
            'arm_joint1', 'arm_joint2', 'arm_joint3',
            'arm_joint4', 'arm_joint5', 'arm_joint6'
        ]
        
        # Control parameters
        self.speed = 0.02  # radians per update
        self.deadzone = 0.1
        
        # Timer for publishing joint states
        self.timer = self.create_timer(0.05, self.publish_joint_states)
        
        self.get_logger().info('Joy teleop started!')
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
        self.joint_pub.publish(msg)
    
    def joy_callback(self, msg):
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
    rclpy.shutdown()


if __name__ == '__main__':
    main()
