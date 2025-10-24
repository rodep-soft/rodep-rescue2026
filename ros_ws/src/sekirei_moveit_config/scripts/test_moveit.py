#!/usr/bin/env python3
"""
Test MoveIt2 planning directly using MoveGroupInterface
This bypasses RViz2 GUI issues
"""
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import PoseStamped
import sys

def main():
    rclpy.init()
    
    # Create MoveItPy instance
    print("Initializing MoveIt2...")
    moveit = MoveItPy(node_name="test_moveit")
    
    # Get the arm planning component
    arm = moveit.get_planning_component("sekirei_arm")
    print(f"✓ Planning component 'sekirei_arm' loaded")
    
    # Get current state
    robot_model = moveit.get_robot_model()
    robot_state = moveit.get_planning_scene_monitor().state_monitor.get_current_state()
    print(f"✓ Current robot state loaded")
    
    # Print current end-effector position
    from moveit.core.kinematic_constraints import construct_joint_constraint
    arm_joints = robot_state.get_joint_positions("sekirei_arm")
    print(f"\nCurrent joint positions:")
    joint_names = robot_model.get_joint_model_group("sekirei_arm").get_active_joint_model_names()
    for name, pos in zip(joint_names, arm_joints):
        print(f"  {name}: {pos:.4f} rad ({pos * 180 / 3.14159:.2f}°)")
    
    # Get current end-effector pose
    ee_link = robot_model.get_joint_model_group("sekirei_arm").get_link_model_names()[-1]
    ee_pose = robot_state.get_frame_transform(ee_link)
    print(f"\nCurrent end-effector pose ({ee_link}):")
    print(f"  Position: x={ee_pose.translation()[0]:.3f}, y={ee_pose.translation()[1]:.3f}, z={ee_pose.translation()[2]:.3f}")
    
    # Set a target pose (slightly different from current)
    if len(sys.argv) >= 4:
        target_x = float(sys.argv[1])
        target_y = float(sys.argv[2])
        target_z = float(sys.argv[3])
    else:
        # Default: move 5cm forward
        target_x = ee_pose.translation()[0] + 0.05
        target_y = ee_pose.translation()[1]
        target_z = ee_pose.translation()[2]
    
    print(f"\nTarget pose:")
    print(f"  Position: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}")
    
    # Set the goal
    arm.set_goal_state(
        pose_stamped_msg=PoseStamped(
            header={'frame_id': 'base_link'},
            pose={
                'position': {'x': target_x, 'y': target_y, 'z': target_z},
                'orientation': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}
            }
        ),
        pose_link=ee_link
    )
    
    # Plan
    print("\nPlanning...")
    plan_result = arm.plan()
    
    if plan_result:
        print("✓ Planning successful!")
        trajectory = plan_result.trajectory
        print(f"  Trajectory points: {len(trajectory.joint_trajectory.points)}")
        print(f"  Duration: {trajectory.joint_trajectory.points[-1].time_from_start.sec + trajectory.joint_trajectory.points[-1].time_from_start.nanosec / 1e9:.3f} seconds")
        
        print("\nFinal joint positions:")
        final_point = trajectory.joint_trajectory.points[-1]
        for name, pos in zip(trajectory.joint_trajectory.joint_names, final_point.positions):
            if 'arm_joint' in name:
                print(f"  {name}: {pos:.4f} rad ({pos * 180 / 3.14159:.2f}°)")
        
        return True
    else:
        print("✗ Planning failed!")
        return False

if __name__ == '__main__':
    try:
        success = main()
        rclpy.shutdown()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        rclpy.shutdown()
        sys.exit(1)
