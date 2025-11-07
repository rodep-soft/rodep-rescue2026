cmake_minimum_required(VERSION 3.8)
project(crawler_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

<<<<<<< HEAD
# Require C++20 for this package (uses std::span and std::bit_cast)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

=======
>>>>>>> f499368714c2dfc6a13d5618a4b690fefd245a3c
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(custom_interfaces REQUIRED)

# 既存のドライバー
add_executable(crawler_driver_node src/crawler_driver.cpp)
ament_target_dependencies(crawler_driver_node rclcpp std_msgs geometry_msgs custom_interfaces)

# Roboclawライブラリ（共通）
add_library(roboclaw_driver src/roboclaw_driver.cpp)
target_include_directories(roboclaw_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(roboclaw_driver rclcpp)

# V2ドライバー（シンプルな実装）
add_executable(crawler_driver_v2_node src/crawler_driver_v2.cpp)
target_link_libraries(crawler_driver_v2_node roboclaw_driver)
ament_target_dependencies(crawler_driver_v2_node rclcpp std_msgs custom_interfaces)

# V3ドライバー（改善版・推奨）
add_executable(crawler_driver_v3_node src/crawler_driver_v3.cpp)
target_link_libraries(crawler_driver_v3_node roboclaw_driver)
ament_target_dependencies(crawler_driver_v3_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros)

install(TARGETS
  crawler_driver_node
  crawler_driver_v2_node
  crawler_driver_v3_node
  roboclaw_driver
  DESTINATION lib/${PROJECT_NAME})

# YAMLファイルのインストール
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)


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

ament_package()
