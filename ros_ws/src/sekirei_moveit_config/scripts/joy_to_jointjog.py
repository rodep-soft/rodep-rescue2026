#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from control_msgs.msg import JointJog

class JoyToJointJog(Node):
    def __init__(self):
        super().__init__('joy_to_jointjog')
        self.publisher_ = self.create_publisher(JointJog, '/servo_node/speed_units_cmd', 10)
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        # ジョイント名はロボットに合わせて修正してください
        self.joint_names = [
            'arm_joint1',
            'arm_joint2',
            'arm_joint3',
            'arm_joint4',
            'arm_joint5',
            'arm_joint6',
        ]
        # 軸割り当て例: 左スティック上下/左右, 右スティック上下/左右, L2/R2
        self.axis_map = [0, 1, 3, 4, 2, 5]  # 必要に応じて修正
        self.scale = 0.5  # 速度スケール

    def joy_callback(self, msg):
        jog = JointJog()
        jog.header.stamp = self.get_clock().now().to_msg()
        jog.header.frame_id = ''
        jog.joint_names = self.joint_names
        jog.velocities = [0.0] * len(self.joint_names)
        for i, axis in enumerate(self.axis_map):
            if i < len(jog.velocities) and axis < len(msg.axes):
                jog.velocities[i] = msg.axes[axis] * self.scale
        self.publisher_.publish(jog)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToJointJog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
