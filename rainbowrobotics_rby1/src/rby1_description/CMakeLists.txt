cmake_minimum_required(VERSION 3.8)
project(rby1_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/sdk_urdf_to_ros.py DESTINATION lib/${PROJECT_NAME})

ament_package()
