#!/usr/bin/env python3
"""
Named Poses Feature for Sekirei Arm

This script demonstrates the 8 preset poses defined in sekirei.srdf.
These poses can be used directly in RViz2's MotionPlanning plugin.

Available Named Poses:
- home: All joints at 0° (neutral position)
- ready: Ready position (10° on joints 2,3,4)
- up: Arm pointing upward
- forward: Arm extended forward
- compact: Compact folded position
- left: Arm pointing left
- right: Arm pointing right
- back: Arm pointing backward

How to Use in RViz2:
1. Launch: ros2 launch sekirei_moveit_config demo.launch.py
2. In RViz2's MotionPlanning panel:
   - Go to "Planning" tab
   - Under "Select Goal State", choose "Select a group state from the list"
   - Pick one of the 8 poses from the dropdown
   - Click "Plan" then "Execute"

Command Line Usage:
    ros2 run sekirei_moveit_config move_to_named_pose.py <pose_name>

Example:
    ros2 run sekirei_moveit_config move_to_named_pose.py home

Note: This script lists available poses.
      For actual execution, use RViz2 GUI or implement with moveit_commander.
"""

import sys


def main():
    poses = {
        'home': 'All joints at 0° (neutral starting position)',
        'ready': 'Ready position with slight joint angles',
        'up': 'Arm pointing upward',
        'forward': 'Arm extended forward',
        'compact': 'Compact folded position for transport',
        'left': 'Arm pointing to the left side',
        'right': 'Arm pointing to the right side',
        'back': 'Arm pointing backward',
    }

    if len(sys.argv) < 2:
        print('\n📍 Named Poses for Sekirei Arm')
        print('=' * 50)
        print('\nAvailable poses:')
        for name, desc in poses.items():
            print(f'  • {name:10s} - {desc}')
        print('\n✨ How to use:')
        print('  1. RViz2 GUI (Recommended):')
        print('     - Launch: ros2 launch sekirei_moveit_config demo.launch.py')
        print('     - In MotionPlanning panel → Planning tab')
        print('     - Select Goal State → "group state from list"')
        print('     - Choose pose → Plan → Execute')
        print('\n  2. Command line:')
        print('     ros2 run sekirei_moveit_config move_to_named_pose.py <pose_name>')
        print('\n' + '=' * 50)
        return 0

    target = sys.argv[1]

    if target in poses:
        print(f'\n✓ Pose "{target}" is defined')
        print(f'  Description: {poses[target]}')
        print(f'\n  To execute in RViz2:')
        print(f'    1. Open MotionPlanning panel')
        print(f'    2. Planning tab → Select Goal State')
        print(f'    3. Choose "{target}" from dropdown')
        print(f'    4. Click Plan, then Execute\n')
        return 0
    else:
        print(f'\n✗ Error: Unknown pose "{target}"')
        print(f'  Available: {", ".join(poses.keys())}\n')
        return 1


if __name__ == '__main__':
    sys.exit(main())
