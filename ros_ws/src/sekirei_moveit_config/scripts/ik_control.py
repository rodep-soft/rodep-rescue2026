#!/usr/bin/env python3
"""
Sekirei Arm IK Control Script
エンドエフェクタの目標位置(x,y,z)を指定して、IKで関節角度を計算して動かす
"""
import rclpy
from rclpy.node import Node
from moveit_py import MoveItPy
from geometry_msgs.msg import PoseStamped
import sys


def main():
    rclpy.init()

    # MoveItPy の初期化
    moveit = MoveItPy(node_name="sekirei_ik_control")

    # アームのプランニンググループを取得
    arm = moveit.get_planning_component("sekirei_arm")

    print("\n" + "="*60)
    print("Sekirei Arm IK Control")
    print("="*60)
    print("アームの先端を指定した位置(x, y, z)に動かします")
    print("="*60)

    while True:
        print("\n目標位置を入力してください:")
        try:
            x = float(input("  X座標 (m): "))
            y = float(input("  Y座標 (m): "))
            z = float(input("  Z座標 (m): "))
        except (ValueError, EOFError):
            print("\n終了します")
            break

        # 目標位置を設定
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z
        pose_goal.pose.orientation.w = 1.0  # 姿勢はデフォルト

        print(f"\n目標位置: ({x}, {y}, {z})")
        print("IK を計算中...")

        # IKを使って目標位置に移動するプランを作成
        arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="arm6_link")

        # プランニング
        plan_result = arm.plan()

        if plan_result:
            print("✓ IK解が見つかりました！")
            print("プランを実行しますか? (y/n): ", end='')

            answer = input().strip().lower()
            if answer == 'y':
                # プランを実行
                robot = moveit.get_robot_model()
                moveit.execute(plan_result.trajectory, blocking=True)
                print("✓ 移動完了!")
            else:
                print("キャンセルしました")
        else:
            print("✗ IK解が見つかりませんでした")
            print("  別の位置を試してください")

    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n終了します")
    except Exception as e:
        print(f"エラー: {e}")
        import traceback
        traceback.print_exc()
