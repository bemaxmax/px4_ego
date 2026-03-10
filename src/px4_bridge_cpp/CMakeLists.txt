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
find_package(px4_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(cmd_vel_to_pos_cmd src/cmd_vel_to_pos_cmd.cpp)
ament_target_dependencies(
  cmd_vel_to_pos_cmd
  geometry_msgs
  px4_msgs
  quadrotor_msgs
  rclcpp
)

add_executable(lidar_gz_bridge src/lidar_gz_bridge.cpp)
ament_target_dependencies(
  lidar_gz_bridge
  rclcpp
  sensor_msgs
)

add_executable(depth_gz_bridge src/depth_gz_bridge.cpp)
ament_target_dependencies(
  depth_gz_bridge
  rclcpp
  sensor_msgs
)

install(
  TARGETS
    cmd_vel_to_pos_cmd
    lidar_gz_bridge
    depth_gz_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
