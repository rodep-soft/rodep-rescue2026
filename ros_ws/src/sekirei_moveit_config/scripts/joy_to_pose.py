#!/usr/bin/env python3
"""Joystick -> PoseStamped bridge for moveit_servo

Maps joystick axes to incremental pose target commands published to
`/servo_node/pose_target_cmds`. Optionally switches servo command type via
`/servo_node/switch_command_type` service (ensure the integer value matches
the servo's supported enum, e.g. 'unitless' on some installations).

Usage:
  . install/setup.bash
  ros2 run joy joy_node   # or run your joystick driver
  python3 scripts/joy_to_pose.py

Parameters (ROS params on the node):
  rate: publish rate in Hz (default 10)
  linear_scale: meters per axis unit per second
  angular_scale: radians per axis unit per second
  switch_service: whether to call the switch_command_type service (default: true)
  pose_command_type_value: integer command type value for pose mode (default: 2)

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from rclpy.duration import Duration
from rclpy.time import Time

# Service type used by repo's joy_to_jointjog.py
from moveit_msgs.srv import ServoCommandType

import math


def quaternion_from_euler(roll, pitch, yaw):
    # simple conversion
    qx = math.sin(roll/2.0)*math.cos(pitch/2.0)*math.cos(yaw/2.0) - math.cos(roll/2.0)*math.sin(pitch/2.0)*math.sin(yaw/2.0)
    qy = math.cos(roll/2.0)*math.sin(pitch/2.0)*math.cos(yaw/2.0) + math.sin(roll/2.0)*math.cos(pitch/2.0)*math.sin(yaw/2.0)
    qz = math.cos(roll/2.0)*math.cos(pitch/2.0)*math.sin(yaw/2.0) - math.sin(roll/2.0)*math.sin(pitch/2.0)*math.cos(yaw/2.0)
    qw = math.cos(roll/2.0)*math.cos(pitch/2.0)*math.cos(yaw/2.0) + math.sin(roll/2.0)*math.sin(pitch/2.0)*math.sin(yaw/2.0)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


class JoyToPose(Node):
    def __init__(self):
        super().__init__('joy_to_pose')

        self.declare_parameter('rate', 10)
        self.declare_parameter('linear_scale', 0.05)   # m per axis unit per second
        self.declare_parameter('angular_scale', 0.5)   # rad per axis unit per second
        self.declare_parameter('switch_service', True)
        self.declare_parameter('pose_command_type_value', 2)
        self.declare_parameter('frame_id', 'base_link')

        # workspace/clamp limits to avoid sending targets that drive the arm
        # into singular or unreachable configurations
        self.declare_parameter('min_x', 0.2)
        self.declare_parameter('max_x', 0.7)
        self.declare_parameter('min_y', -0.4)
        self.declare_parameter('max_y', 0.4)
        self.declare_parameter('min_z', 0.05)
        self.declare_parameter('max_z', 0.5)

        self.rate = self.get_parameter('rate').get_parameter_value().integer_value
        self.linear_scale = self.get_parameter('linear_scale').get_parameter_value().double_value
        self.angular_scale = self.get_parameter('angular_scale').get_parameter_value().double_value
        self.switch_service = self.get_parameter('switch_service').get_parameter_value().bool_value
        self.pose_command_type_value = self.get_parameter('pose_command_type_value').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.min_x = self.get_parameter('min_x').get_parameter_value().double_value
        self.max_x = self.get_parameter('max_x').get_parameter_value().double_value
        self.min_y = self.get_parameter('min_y').get_parameter_value().double_value
        self.max_y = self.get_parameter('max_y').get_parameter_value().double_value
        self.min_z = self.get_parameter('min_z').get_parameter_value().double_value
        self.max_z = self.get_parameter('max_z').get_parameter_value().double_value
        # deadzone to ignore small joystick noise
        self.declare_parameter('deadzone', 0.05)
        self.deadzone = self.get_parameter('deadzone').get_parameter_value().double_value
        # how often to log a clamp warning (seconds)
        self.declare_parameter('clamp_warn_interval', 1.0)
        self.clamp_warn_interval = self.get_parameter('clamp_warn_interval').get_parameter_value().double_value
        self.last_clamp_warn_time = None

        # publisher to servo pose topic
        self.pose_pub = self.create_publisher(PoseStamped, '/servo_node/pose_target_cmds', 10)

        # subscription to joystick
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)

        # internal state: current target pose
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = self.frame_id
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        # initialize to a reasonable default; if you have robot's current ee pose, you can set it
        self.current_pose.pose.position.x = 0.35
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 0.2
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.current_pose.pose.orientation = quaternion_from_euler(self.current_roll, self.current_pitch, self.current_yaw)

        # last joystick message and timer
        self.last_joy = None
        self.timer = self.create_timer(1.0/self.rate, self.timer_cb)
        # track last published pose to avoid publishing the same clamped pose repeatedly
        self.declare_parameter('publish_epsilon', 1e-4)
        self.publish_epsilon = self.get_parameter('publish_epsilon').get_parameter_value().double_value
        self.last_published_pose = None

        # Optionally switch servo to pose mode via service
        if self.switch_service:
            self.cli = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
            self.get_logger().info('Waiting for /servo_node/switch_command_type service...')
            if self.cli.wait_for_service(timeout_sec=5.0):
                req = ServoCommandType.Request()
                req.command_type = int(self.pose_command_type_value)
                fut = self.cli.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
                if fut.result() and fut.result().success:
                    self.get_logger().info(f'servo_node command type set (requested value: {self.pose_command_type_value})')
                else:
                    self.get_logger().warn('Failed to set servo_node command type (service returned false)')
            else:
                self.get_logger().warn('/servo_node/switch_command_type service not available; continuing without switching')

    def joy_cb(self, msg: Joy):
        self.last_joy = msg

    def timer_cb(self):
        if self.last_joy is None:
            return
    # Map axes to increments. Axis mapping may vary per joystick.
        axes = self.last_joy.axes
        # simple default mapping: axes[0]=left/right -> y, axes[1]=forward/back -> x, axes[2]=z? , axes[3]=roll or yaw
        # Adjust mapping to your joystick
        x_in = axes[1] if len(axes) > 1 else 0.0
        y_in = axes[0] if len(axes) > 0 else 0.0
        z_in = axes[3] if len(axes) > 3 else 0.0
        yaw_in = axes[2] if len(axes) > 2 else 0.0

        # Apply deadzone to avoid drift/noise causing continuous clamping
        if abs(x_in) < self.deadzone:
            x_in = 0.0
        if abs(y_in) < self.deadzone:
            y_in = 0.0
        if abs(z_in) < self.deadzone:
            z_in = 0.0
        if abs(yaw_in) < self.deadzone:
            yaw_in = 0.0

        dt = 1.0/self.rate
        # If all inputs are zero after deadzone, skip publishing to reduce spam
        if x_in == 0.0 and y_in == 0.0 and z_in == 0.0 and yaw_in == 0.0:
            return

        self.current_pose.pose.position.x += x_in * self.linear_scale * dt
        self.current_pose.pose.position.y += y_in * self.linear_scale * dt
        self.current_pose.pose.position.z += z_in * self.linear_scale * dt

        # Clamp workspace so we don't command positions outside a safe box
        clamped = False
        if self.current_pose.pose.position.x < self.min_x:
            self.current_pose.pose.position.x = self.min_x
            clamped = True
        if self.current_pose.pose.position.x > self.max_x:
            self.current_pose.pose.position.x = self.max_x
            clamped = True
        if self.current_pose.pose.position.y < self.min_y:
            self.current_pose.pose.position.y = self.min_y
            clamped = True
        if self.current_pose.pose.position.y > self.max_y:
            self.current_pose.pose.position.y = self.max_y
            clamped = True
        if self.current_pose.pose.position.z < self.min_z:
            self.current_pose.pose.position.z = self.min_z
            clamped = True
        if self.current_pose.pose.position.z > self.max_z:
            self.current_pose.pose.position.z = self.max_z
            clamped = True
        if clamped:
            now = self.get_clock().now()
            if self.last_clamp_warn_time is None:
                should_warn = True
            else:
                delta = (now - self.last_clamp_warn_time).nanoseconds / 1e9
                should_warn = delta >= self.clamp_warn_interval
            if should_warn:
                self.get_logger().warn('Target pose clamped to workspace limits to avoid singularity/unreachable target')
                self.last_clamp_warn_time = now

        self.current_yaw += yaw_in * self.angular_scale * dt
        self.current_pose.pose.orientation = quaternion_from_euler(self.current_roll, self.current_pitch, self.current_yaw)

        # Only publish if pose changed more than epsilon to avoid repeated identical commands
        curr = self.current_pose.pose
        curr_tuple = (curr.position.x, curr.position.y, curr.position.z, self.current_yaw)
        should_publish = True
        if self.last_published_pose is not None:
            diffs = [abs(a - b) for a, b in zip(curr_tuple, self.last_published_pose)]
            if max(diffs) < self.publish_epsilon:
                should_publish = False

        if should_publish:
            self.current_pose.header.stamp = self.get_clock().now().to_msg()
            self.pose_pub.publish(self.current_pose)
            self.last_published_pose = curr_tuple


def main(args=None):
    rclpy.init(args=args)
    node = JoyToPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
