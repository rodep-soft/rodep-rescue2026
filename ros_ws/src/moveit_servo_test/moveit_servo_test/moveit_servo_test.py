import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TwistStamped

class ServoTest(Node):
    def __init__(self):
        super().__init__('moveit_servo_test')
        self.publisher_ = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            QoSProfile(depth=10).best_effort()
        )
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.1
        msg.twist.angular.z = 0.1
        self.publisher_.publish(msg)
        self.get_logger().info('Sent servo command')

def main(args=None):
    rclpy.init(args=args)
    servo_test = ServoTest()
    rclpy.spin(servo_test)
    servo_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


