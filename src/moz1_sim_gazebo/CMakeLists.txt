cmake_minimum_required(VERSION 3.8)
project(moz1_sim_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/scene_markers.py scripts/grasp_helper.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
