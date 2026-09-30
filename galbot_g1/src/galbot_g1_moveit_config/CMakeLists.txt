cmake_minimum_required(VERSION 3.8)
project(galbot_g1_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# Tells MoveItConfigsBuilder where the URDF/SRDF live.
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
