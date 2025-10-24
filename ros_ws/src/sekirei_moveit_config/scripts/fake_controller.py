#!/usr/bin/env python3
"""
Fake controller action server for MoveIt2 trajectory execution visualization.
This node accepts trajectory goals and publishes joint states to visualize motion.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import threading
import time


class FakeTrajectoryController(Node):
    def __init__(self):
        super().__init__('fake_trajectory_controller')
        
        # Create action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/fake_arm_controller/follow_joint_trajectory',
            self.execute_callback
        )
        
        # Publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Joint names and current positions (kept updated and periodically published)
        self.joint_names = [
            'arm_joint1', 'arm_joint2', 'arm_joint3',
            'arm_joint4', 'arm_joint5', 'arm_joint6'
        ]
        self._lock = threading.Lock()
        self.current_positions = [0.0 for _ in self.joint_names]

        # Periodic publisher so MoveIt always sees a recent joint_states message
        publish_rate_hz = 30.0
        self.create_timer(1.0 / publish_rate_hz, self._publish_current_state)
        
        # Subscribe to joint commands (e.g., from joystick)
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            '/joint_commands',
            self._joint_command_callback,
            10
        )
        
        self.get_logger().info('Fake trajectory controller started')
        self.get_logger().info('Listening on: /fake_arm_controller/follow_joint_trajectory')
        self.get_logger().info('Listening on: /joint_commands for manual control')
    
    def execute_callback(self, goal_handle):
        """Execute trajectory by publishing interpolated joint states"""
        self.get_logger().info('Received trajectory execution request')
        
        trajectory = goal_handle.request.trajectory
        
        # Publish feedback
        feedback_msg = FollowJointTrajectory.Feedback()
        
        # Track previous time for incremental sleep
        prev_time = 0.0
        
        # Execute trajectory by publishing joint states
        for i, point in enumerate(trajectory.points):
            # Create joint state message
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = trajectory.joint_names
            joint_state.position = list(point.positions)
            
            # Publish joint state
            self.joint_state_pub.publish(joint_state)
            
            # Also update the current_positions so the periodic publisher reflects execution
            with self._lock:
                # Ensure alignment: fill missing joints with previous values if needed
                for idx, name in enumerate(self.joint_names):
                    try:
                        self.current_positions[idx] = joint_state.position[idx]
                    except IndexError:
                        # keep previous value
                        pass
            # Update feedback
            feedback_msg.desired = point
            feedback_msg.actual = point
            goal_handle.publish_feedback(feedback_msg)
            
            # Sleep based on time difference from previous point
            current_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            sleep_duration = current_time - prev_time
            if sleep_duration > 0:
                time.sleep(sleep_duration)
            prev_time = current_time
        
        # Mark as succeeded
        goal_handle.succeed()
        
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        
        self.get_logger().info('Trajectory execution completed successfully')
        return result

    def _publish_current_state(self):
        """Periodic publisher to send the latest joint positions on /joint_states."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        with self._lock:
            msg.position = list(self.current_positions)
        self.joint_state_pub.publish(msg)
    
    def _joint_command_callback(self, msg):
        """Update current positions based on incoming joint commands (e.g., from joystick)."""
        with self._lock:
            for i, name in enumerate(msg.name):
                if name in self.joint_names:
                    idx = self.joint_names.index(name)
                    if i < len(msg.position):
                        self.current_positions[idx] = msg.position[i]


def main(args=None):
    rclpy.init(args=args)
    
    fake_controller = FakeTrajectoryController()
    
    try:
        rclpy.spin(fake_controller)
    except KeyboardInterrupt:
        pass
    
    fake_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
