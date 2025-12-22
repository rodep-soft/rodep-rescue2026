cmake_minimum_required(VERSION 3.8)
project(sekirei_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

add_executable(ee_controller src/ee_controller.cpp)
ament_target_dependencies(ee_controller rclcpp sensor_msgs dynamixel_sdk)
install(TARGETS
  ee_controller
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

  scripts/fake_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
