cmake_minimum_required(VERSION 3.8)
project(urdf_test_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



add_executable(joy_teleop src/joy_teleop.cpp)
ament_target_dependencies(joy_teleop
  rclcpp
  sensor_msgs
  geometry_msgs
  moveit_ros_planning_interface
)


install(TARGETS
  joy_teleop
  DESTINATION lib/${PROJECT_NAME}
)


# CMakeLists.txt に以下を追加または確認してください
# Launchファイル群をインストール
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.py"
)


# 👇 これが最重要：urdfディレクトリ全体を、share/<package>/urdf にコピー
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# meshesディレクトリをインストール
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# rvizディレクトリをインストール
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
