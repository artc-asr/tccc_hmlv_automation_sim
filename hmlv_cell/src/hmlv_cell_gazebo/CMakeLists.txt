cmake_minimum_required(VERSION 3.8)
project(hmlv_cell_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config robots DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/transfer_scene.py scripts/transfer_demo.py scripts/record_run.py
        scripts/reach_sweep.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
