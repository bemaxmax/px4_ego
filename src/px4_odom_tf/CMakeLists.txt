cmake_minimum_required(VERSION 3.8)
project(px4_odom_tf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(px4_odom_tf_publisher src/px4_odom_tf_publisher.cpp)
ament_target_dependencies(
  px4_odom_tf_publisher
  geometry_msgs
  px4_msgs
  rclcpp
  rosgraph_msgs
  tf2_ros
)

install(TARGETS
  px4_odom_tf_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
