cmake_minimum_required(VERSION 3.8)
project(pointcloud_test_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(pointcloud_test_publisher
  src/pointcloud_test_publisher.cpp
)
ament_target_dependencies(pointcloud_test_publisher
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  pointcloud_test_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
