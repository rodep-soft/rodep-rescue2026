#!/usr/bin/env python3
"""
Simple script to test IK control via MoveIt2 without RViz2 interactive markers
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import sys
import time

class IKTester(Node):
    def __init__(self):
        super().__init__('ik_tester')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.joint_state = None
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
    def joint_state_callback(self, msg):
        self.joint_state = msg
        
    def wait_for_joint_states(self):
        """Wait for joint states to be available"""
        self.get_logger().info('Waiting for joint states...')
        timeout = 5.0
        start = time.time()
        while self.joint_state is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.joint_state is None:
            self.get_logger().error('Failed to receive joint states')
            return False
        
        self.get_logger().info(f'Received joint states: {len(self.joint_state.name)} joints')
        return True
        
    def compute_ik(self, x, y, z):
        """Compute IK for target pose"""
        # Wait for joint states first
        if not self.wait_for_joint_states():
            return False
            
        self.get_logger().info(f'Waiting for IK service...')
        self.ik_client.wait_for_service()
        
        # Create IK request
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'sekirei_arm'
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 5
        
        # Set target pose
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # Default orientation (pointing down)
        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        
        request.ik_request.pose_stamped = pose
        
        # Set current robot state
        robot_state = RobotState()
        robot_state.joint_state = self.joint_state
        request.ik_request.robot_state = robot_state
        
        self.get_logger().info(f'Computing IK for: x={x}, y={y}, z={z}')
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        
        if response.error_code.val == 1:  # SUCCESS
            self.get_logger().info('✓ IK solution found!')
            joint_names = response.solution.joint_state.name
            joint_positions = response.solution.joint_state.position
            
            print("\nJoint solution:")
            for name, pos in zip(joint_names, joint_positions):
                if 'arm_joint' in name:
                    print(f"  {name}: {pos:.4f} rad ({pos * 180 / 3.14159:.2f}°)")
            return True
        else:
            error_codes = {
                -1: "PLANNING_FAILED",
                -2: "INVALID_MOTION_PLAN",
                -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
                -4: "CONTROL_FAILED",
                -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
                -6: "TIMED_OUT",
                -7: "PREEMPTED",
                -10: "START_STATE_IN_COLLISION",
                -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
                -12: "GOAL_IN_COLLISION",
                -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
                -14: "GOAL_CONSTRAINTS_VIOLATED",
                -15: "INVALID_GROUP_NAME",
                -16: "INVALID_GOAL_CONSTRAINTS",
                -21: "NO_IK_SOLUTION",
                -31: "FRAME_TRANSFORM_FAILURE",
            }
            error_msg = error_codes.get(response.error_code.val, f"UNKNOWN ({response.error_code.val})")
            self.get_logger().error(f'✗ IK failed: {error_msg}')
            return False

def main():
    rclpy.init()
    
    if len(sys.argv) < 4:
        print("Usage: test_ik.py <x> <y> <z>")
        print("Example: test_ik.py 0.3 0.0 0.3")
        print("\nCurrent arm position is approximately: x=0.29, y=0.0, z=0.26")
        print("Try positions within reach: x=0.2-0.4, y=-0.2-0.2, z=0.1-0.5")
        sys.exit(1)
    
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    
    tester = IKTester()
    success = tester.compute_ik(x, y, z)
    
    if success:
        print(f"\n✓ IK solution exists for ({x}, {y}, {z})")
    else:
        print(f"\n✗ No IK solution for ({x}, {y}, {z})")
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
