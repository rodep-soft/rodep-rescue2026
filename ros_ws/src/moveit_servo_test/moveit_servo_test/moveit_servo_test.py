import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class ServoUnitlessTest(Node):
    def __init__(self):
        super().__init__("servo_unitless_test")

        qos_joint = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        qos_twist = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # JointJog publisher
        self.joint_publisher = self.create_publisher(
            JointJog, "/servo_node/delta_joint_cmds", qos_joint
        )

        # TwistStamped publisher (Cartesian制御用)
        self.twist_publisher = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", qos_twist
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.joint_names = [
            "arm_joint1",
            "arm_joint2",
            "arm_joint3",
            "arm_joint4",
            "arm_joint5",
            "arm_joint6",
        ]

    def timer_callback(self):
        # ===== JointJog =====
        joint_msg = JointJog()
        now = self.get_clock().now().to_msg()
        joint_msg.header.stamp = now
        joint_msg.header.frame_id = "base_link"
        joint_msg.joint_names = self.joint_names
        joint_msg.velocities = [0.1] * len(self.joint_names)
        joint_msg.velocities[2] = 0.0
        joint_msg.velocities[4] = 0.0
        joint_msg.duration = 0.1
        self.joint_publisher.publish(joint_msg)
        self.get_logger().info(f"Sent JointJog command at {now.sec}.{now.nanosec}")

        # ===== TwistStamped =====
        twist_msg = TwistStamped()
        twist_msg.header.stamp = now
        twist_msg.header.frame_id = "base_link"
        # Cartesianの速度指令
        twist_msg.twist.linear.x = 0.0  # x方向に0.05 m/s
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = -1.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0  # z軸回転0.05 rad/s
        self.twist_publisher.publish(twist_msg)
        self.get_logger().info(f"Sent TwistStamped command at {now.sec}.{now.nanosec}")


def main(args=None):
    rclpy.init(args=args)
    node = ServoUnitlessTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
