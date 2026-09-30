cmake_minimum_required(VERSION 3.8)
project(rby1_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/rby1_demo.py scripts/world_markers.py DESTINATION lib/${PROJECT_NAME})

ament_package()
