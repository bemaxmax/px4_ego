cmake_minimum_required(VERSION 3.8)
project(px4_bridge_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_executable(cmd_vel_to_pos_cmd src/cmd_vel_to_pos_cmd.cpp)
ament_target_dependencies(
  cmd_vel_to_pos_cmd
  geometry_msgs
  px4_msgs
  quadrotor_msgs
  rclcpp
)

add_executable(depth_gz_bridge src/depth_gz_bridge.cpp)
ament_target_dependencies(
  depth_gz_bridge
  rclcpp
  sensor_msgs
)

add_executable(lidar_points_to_world src/lidar_points_to_world.cpp)
ament_target_dependencies(
  lidar_points_to_world
  nav_msgs
  rclcpp
  sensor_msgs
)

add_executable(gazebo_truth_to_visual_odometry src/gazebo_truth_to_visual_odometry.cpp)
ament_target_dependencies(
  gazebo_truth_to_visual_odometry
  nav_msgs
  px4_msgs
  rclcpp
)
target_link_libraries(
  gazebo_truth_to_visual_odometry
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)

install(
  TARGETS
    cmd_vel_to_pos_cmd
    depth_gz_bridge
    lidar_points_to_world
    gazebo_truth_to_visual_odometry
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
