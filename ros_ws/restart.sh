#!/usr/bin/bash

pkill -f ros
make
. install/setup.bash
ros2 launch sekirei_moveit_config arm_controller.py > log.txt
