import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from control_msgs.msg import JointJog

class ServoUnitlessTest(Node):
    def __init__(self):
        super().__init__('servo_unitless_test')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # unitless コマンドは JointJog メッセージで送る
        self.publisher_ = self.create_publisher(
            JointJog,
            '/servo_node/delta_joint_cmds',
            qos
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        # positions は空で unitless モードでは velocity だけ使う
        msg.velocities = [0.1] * 6  # 6軸ロボットなら全部同じ速度で
        msg.duration = 0.1  # 0.1秒だけ進める
        self.publisher_.publish(msg)
        self.get_logger().info('Sent unitless servo command')

def main(args=None):
    rclpy.init(args=args)
    node = ServoUnitlessTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
