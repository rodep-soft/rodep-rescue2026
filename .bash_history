fish
exit
source /opt/ros/jazzy/setup.bash
colcon build
ls
clera
fish
exit
fish
exit
ros2
ls
source /opt/ros/jazzy/setup.bash
make
ls
make
sudo apt update
sudo apt install ros-${ROS_DISTRO}-osrf-testing-tools-cpp
ls
make
ls
cd ..
ls
cd microros_ws/
ls
colcon build
cd src
ls
cd micro_ros_setup/
ls
cd ..
ls
cd ..
ls
cd ..
ls
cds c
cd review/
ls
cd ..
ls
cd ros_ws/
ls
cd firmware/
ls
cd ..
ls
cd ..
ls
source install/setup.bash
ls
cd ros_ws/
ls
source install/setup.bash
ls
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
bg
ros2 topic list
ros2 node list
ls /dev/ttyA*
lsof /dev/ttyACM0
jobs
kill %1
lsof /dev/ttyACM0
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
bg
ros2 node list
ros2 topic list
jobs
kill %1
ls
cd firmware/
ls
cd ..
ls
exit
fish
exit
fish
exit
fish
fish
. install/setup.bash
fish
. install/setup.bash
fish
. install/setup.bsah
. install/setup.bash
fish
. install/setup.bash
fish
. install/setup.bash
fish
ls
cd ..
ls
exit
ls
ros2 launch sekirei_moveit_config demo.launch.py &> log.txt
make
. install/setup.bash
ros2 launch sekirei_moveit_config demo.launch.py
make
. install/setup.bash
ros2 launch sekirei_moveit_config demo.launch.py
make
. install/setup.bsah
. install/setup.bash
ros2 launch sekirei_moveit_config demo.launch.py
ros2 launch sekirei_moveit_config demo.launch.py &> /dev/null
bg
ros2 topic list | grep interactive
ros2 topic echo /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update --once
ros2 topic list | grep interactive
jobs
kill %1
cd /root/ros_ws && chmod +x src/sekirei_moveit_config/scripts/test_ik.py
ls
ros2 launch sekirei_moveit_config demo.launch.py &> /dev/null &
python3 src/sekirei_moveit_config/scripts/test_ik.py 0.3 0.0 0.3
python3 src/sekirei_moveit_config/scripts/test_ik.py 0.29 0.05 0.26
ros2 service list | grep ik
python3 src/sekirei_moveit_config/scripts/test_ik.py 0.29 0.05 0.26
odeppc:~/ros_ws# python3 src/sekirei_moveit_config/scripts/test_ik.py 0.29 0.05 0.26
[INFO] [1761295278.523716825] [ik_tester]: Waiting for joint states...
[INFO] [1761295279.981545482] [ik_tester]: Received joint states: 6 joints
[INFO] [1761295279.981801156] [ik_tester]: Waiting for IK service...
[INFO] [1761295279.982394177] [ik_tester]: Computing IK for: x=0.29, y=0.05, z=0.26
[ERROR] [1761295284.989685188] [ik_tester]: ✗ IK failed: FRAME_TRANSFORM_FAILURE
✗ No IK solution for (0.29, 0.05, 0.26)
root@rodeppc:~/ros_ws# 
ros2 run tf2_ros tf2_echo base_link arm6_link
ros2 topic echo /display_planned_path --once
ros2 topic list | grep -i plan
ros2 topic list | grep display
chmod +x src/sekirei_moveit_config/scripts/test_moveit.py
python3 src/sekirei_moveit_config/scripts/test_moveit.py
ros2 topic echo /display_planned_path --once
ls
cat requirements.txt 
exit
bash
exit
colcon build --packages-select sekirei_moveit_config
source install/setup.bash
ros2 launch sekirei_moveit_config demo.launch.py
cat /tmp/launch_params_giox69az | grep -B 5 -A 15 "planning_pipelines"
colcon build --packages-select sekirei_moveit_config
source install/setup.bash
ros2 launch sekirei_moveit_config demo.launch.py
colcon build --packages-select sekirei_moveit_config
source install/setup.bash
ros2 launch sekirei_moveit_config demo.launch.py
make
. install/setup.bash
ros2 launch sekirei_moveit_config demo.launch.py
ls -lt /tmp/launch_params_* 2>/dev/null | head -5
cat /tmp/launch_params_4kl2nu9k 
cat /tmp/launch_params_dueh9sni 2>/dev/null || (ls -t /tmp/launch_params_* | head -2 | tail -1 | xargs cat)
source install/setup.bash && ros2 launch sekirei_moveit_config demo.launch.py 2>&1 | grep -A 20 "move_group.*terminate\|move_group.*what()"
make
. install/setup.bash
ros2 launch sekirei_moveit_config demo.launch.py &> log.txt
make
. install/setup.bash
ros2 launch sekirei_moveit_config demo.launch.py &> log.txt
make
. install/setup.bash
fish
exit
. install/setup.bash
fish
exit
. install/setup.bash
fish
exit
. install/setup.bash
fish
exit
. install/setup.bash
fish
exit
. install/setup.bash
fish
exit
. install/setup.bash
fish
exit
ls
make find
cat Makefile 
ls
exit
make
ls
make clean
make
. install/setup.bash
fish
exit
fish
exit
. install/setup.bash
ls
cd src
ls
cd sekirei_moveit_config/
ls
cd launch/
s
ls
ros2 launch demo.launch.py 
ls
cd ..
ls
cd ..
ls
cd launch
ls
cd camera/
ls
ros2 launch ffmpeg_compression_test.py 
bg
rqt
ros2 topic olist
ros2 topic list
ros2 topic info /image_raw/compressed
ros2 topic info /image_raw/compressed --verbose
rqt
lsusb
ls /dev/tty*
ls /dev/tty* | grep video
ls /dev/ | fzf
v4l2-ctl --list-devices
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
fihs
fish
