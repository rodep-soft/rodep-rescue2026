import rclpy
from control_msgs.msg import JointJog
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class ServoUnitlessTest(Node):
    def __init__(self):
        super().__init__("servo_unitless_test")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # unitless コマンドは JointJog メッセージで送る
        self.publisher_ = self.create_publisher(
            JointJog, "/servo_node/delta_joint_cmds", qos
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.frame_id = "base_link"

        # ジョイント名を固定（自分のロボットに合わせる）
        self.joint_names = [
            "arm_joint1",
            "arm_joint2",
            "arm_joint3",
            "arm_joint4",
            "arm_joint5",
            "arm_joint6",
        ]

    def timer_callback(self):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        msg.velocities = [0.1] * len(self.joint_names)  # 6軸全部に同じ速度
        msg.velocities[2] = 0.0
        msg.duration = 0.1  # 0.1秒だけ進める
        self.publisher_.publish(msg)
        self.get_logger().info("Sent unitless servo command")


def main(args=None):
    rclpy.init(args=args)
    node = ServoUnitlessTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
