#!/usr/bin/env python3
"""
MoveIt Commander を使った簡単な IK 制御
"""
import sys
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped


class MoveGroupActionClient(Node):
    def __init__(self):
        super().__init__('move_group_action_client')
        self._action_client = ActionClient(self, MoveGroup, '/move_action')

    def send_goal(self, target_pose):
        """目標姿勢に移動"""
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'sekirei_arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.workspace_parameters.header.frame_id = 'base_link'

        # 目標姿勢を設定
        pose_goal = PositionConstraint()
        pose_goal.header.frame_id = 'base_link'
        pose_goal.link_name = 'arm6_link'
        pose_goal.target_point_offset.x = 0.0
        pose_goal.target_point_offset.y = 0.0
        pose_goal.target_point_offset.z = 0.0

        # 位置制約
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.001]  # 1mm の球
        pose_goal.constraint_region.primitives = [primitive]

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_link'
        pose_stamped.pose = target_pose
        pose_goal.constraint_region.primitive_poses = [pose_stamped.pose]
        pose_goal.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [pose_goal]
        goal_msg.request.goal_constraints = [constraints]

        self.get_logger().info('Sending goal...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        self.get_logger().info('Goal accepted')

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('✓ Motion planning succeeded!')
            return True
        else:
            self.get_logger().error(f'✗ Motion planning failed: {result.error_code.val}')
            return False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Planning state: {feedback.state}')


def main():
    rclpy.init()

    client = MoveGroupActionClient()

    print("\n" + "="*60)
    print("MoveIt2 IK Control for Sekirei Arm")
    print("="*60)
    print("目標位置を入力してください")
    print("="*60 + "\n")

    try:
        while True:
            try:
                x = float(input("X座標 [m] (例: 0.3): "))
                y = float(input("Y座標 [m] (例: 0.0): "))
                z = float(input("Z座標 [m] (例: 0.5): "))

                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = z
                pose.orientation.w = 1.0  # デフォルトの姿勢

                print(f"\n目標位置: ({x}, {y}, {z})")

                success = client.send_goal(pose)

                if success:
                    print("✓ プランニング成功！\n")
                else:
                    print("✗ プランニング失敗。別の位置を試してください。\n")

            except ValueError:
                print("数値を入力してください")
            except KeyboardInterrupt:
                break

    except KeyboardInterrupt:
        pass
    finally:
        print("\n終了します")
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
