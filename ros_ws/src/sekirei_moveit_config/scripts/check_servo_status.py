#!/usr/bin/env python3
"""
Check Dynamixel servo status and send test commands
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time

class ServoChecker(Node):
    def __init__(self):
        super().__init__('servo_checker')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        self.command_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10)
        
        self.latest_state = None
        self.get_logger().info('Servo Checker started. Listening to /joint_states...')
    
    def joint_state_callback(self, msg):
        self.latest_state = msg
        self.get_logger().info(f'Joint positions: {[f"{p:.3f}" for p in msg.position]}')
        self.get_logger().info(f'Joint velocities: {[f"{v:.3f}" for v in msg.velocity]}')
    
    def send_test_command(self, joint_index, position):
        """Send test command to a specific joint"""
        if self.latest_state is None:
            self.get_logger().warn('No joint state received yet')
            return
        
        cmd = Float64MultiArray()
        cmd.data = list(self.latest_state.position)
        cmd.data[joint_index] = position
        
        self.get_logger().info(f'Sending command: joint {joint_index} to {position:.3f} rad')
        self.command_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    checker = ServoChecker()
    
    try:
        # Wait for first joint state
        time.sleep(2)
        
        # Send test command to joint 0 (move 0.1 rad)
        if checker.latest_state:
            checker.get_logger().info('Sending test command to joint 0...')
            current_pos = checker.latest_state.position[0]
            checker.send_test_command(0, current_pos + 0.1)
        
        rclpy.spin(checker)
    except KeyboardInterrupt:
        pass
    finally:
        checker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
