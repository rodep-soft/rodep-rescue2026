#!/usr/bin/env python3
"""
Simple IK solver for Sekirei arm using MoveIt compute_ik service
MoveIt の compute_ik サービスを使ってIKを計算
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


class SimpleIKClient(Node):
    def __init__(self):
        super().__init__('simple_ik_client')

        # IK サービスクライアント
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )

        # Joint state パブリッシャー
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.get_logger().info('Waiting for IK service...')
        self.ik_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('IK service ready!')

    def compute_ik(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """
        指定した位置(x,y,z)と姿勢(roll,pitch,yaw)に対するIKを計算
        """
        request = GetPositionIK.Request()

        # 目標姿勢を設定
        request.ik_request.group_name = "sekirei_arm"
        request.ik_request.avoid_collisions = True

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # オイラー角からクォータニオンに変換（簡略版）
        from math import sin, cos
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose.pose.orientation.z = cr * cp * sy - sr * sp * cy

        request.ik_request.pose_stamped = pose

        # IKを計算
        self.get_logger().info(f'Computing IK for position: ({x:.3f}, {y:.3f}, {z:.3f})')
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.error_code.val == 1:  # SUCCESS
                self.get_logger().info('✓ IK solution found!')
                return response.solution.joint_state
            else:
                self.get_logger().error(f'✗ IK failed with error code: {response.error_code.val}')
                return None
        else:
            self.get_logger().error('✗ IK service call failed')
            return None

    def publish_joint_state(self, joint_state):
        """
        計算した関節角度をパブリッシュ
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_state.name
        msg.position = joint_state.position
        self.joint_pub.publish(msg)
        self.get_logger().info('Published joint states')


def main():
    rclpy.init()
    node = SimpleIKClient()

    print("\n" + "="*60)
    print("Simple IK Control for Sekirei Arm")
    print("="*60)
    print("エンドエフェクタの目標位置を入力してください")
    print("例: X=0.3, Y=0.0, Z=0.5 で前方の位置")
    print("="*60 + "\n")

    try:
        while True:
            try:
                x = float(input("X座標 [m]: "))
                y = float(input("Y座標 [m]: "))
                z = float(input("Z座標 [m]: "))

                # IKを計算
                joint_state = node.compute_ik(x, y, z)

                if joint_state:
                    print("\n計算された関節角度:")
                    for name, pos in zip(joint_state.name, joint_state.position):
                        print(f"  {name}: {pos:.3f} rad ({pos*57.3:.1f}°)")

                    # 関節角度をパブリッシュして動かす
                    node.publish_joint_state(joint_state)
                    print("\n✓ ロボットが動きます！\n")
                else:
                    print("\n✗ その位置には到達できません\n")

            except ValueError:
                print("数値を入力してください")
            except KeyboardInterrupt:
                break

            print()

    except KeyboardInterrupt:
        pass
    finally:
        print("\n終了します")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
