cmake_minimum_required(VERSION 3.8)
project(sekirei_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(ee_controller src/ee_controller.cpp)
add_executable(joy_teleop src/joy_teleop.cpp)
ament_target_dependencies(ee_controller rclcpp sensor_msgs dynamixel_sdk)
ament_target_dependencies(joy_teleop rclcpp sensor_msgs geometry_msgs dynamixel_sdk moveit_ros_planning_interface  tf2_geometry_msgs)

install(TARGETS
  ee_controller
  joy_teleop
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/joy_teleop.py
  scripts/move_to_named_pose.py
  scripts/moveit_bridge.py
  scripts/joy_to_jointjog.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
