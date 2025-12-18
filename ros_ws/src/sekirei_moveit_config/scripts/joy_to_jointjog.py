#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType

class JoyToJointJog(Node):
    def __init__(self):
        super().__init__('joy_to_jointjog')

        # command_ready は最初に必ず False で初期化
        self.command_ready = False

        # Publisher
        self.publisher_ = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)

        # Subscriber
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # ジョイント名と軸割り当て
        self.joint_names = [
            'arm_joint1',
            'arm_joint2',
            'arm_joint3',
            'arm_joint4',
            'arm_joint5',
            'arm_joint6',
        ]
        self.axis_map = [0, 1, 3, 4, 2, 5]  # 必要に応じて調整
        self.scale = 0.5

        # Servo コマンドタイプ設定
        self.cli = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /servo_node/switch_command_type service...')

        req = ServoCommandType.Request()
        req.command_type = 1  # 1 = Joint
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('servo_node command type set to Joint')
            self.command_ready = True  # サービス成功したら True に
        else:
            self.get_logger().error('Failed to set servo_node command type')

    def joy_callback(self, msg):
        # command_ready が True でない場合は処理しない
        if not self.command_ready:
            return

        jog = JointJog()
        jog.header.stamp = self.get_clock().now().to_msg()
        jog.header.frame_id = ''
        jog.joint_names = self.joint_names
        jog.velocities = [0.0] * len(self.joint_names)

        for i, axis in enumerate(self.axis_map):
            if i < len(jog.velocities) and axis < len(msg.axes):
                jog.velocities[i] = msg.axes[axis] * self.scale

        jog.duration = 0.1  # ←ここを設定、0だと無視される
        self.publisher_.publish(jog)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToJointJog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
