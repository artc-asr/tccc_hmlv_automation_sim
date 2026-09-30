cmake_minimum_required(VERSION 3.8)
project(galbot_g1_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pointcloud_frame_relay src/pointcloud_frame_relay.cpp)
target_compile_features(pointcloud_frame_relay PUBLIC cxx_std_17)
ament_target_dependencies(pointcloud_frame_relay rclcpp sensor_msgs)
install(TARGETS pointcloud_frame_relay DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY urdf config launch worlds rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS scripts/demo_motion.py scripts/world_markers.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
