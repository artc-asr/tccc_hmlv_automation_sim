cmake_minimum_required(VERSION 3.8)
project(hmlv_cell_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)

# The duo cell's belt: fill the pair under the filler, index one station
# (see src/indexing_conveyor.cc), a world plugin.
add_library(hmlv_indexing_conveyor SHARED src/indexing_conveyor.cc)
target_compile_features(hmlv_indexing_conveyor PRIVATE cxx_std_17)
target_link_libraries(hmlv_indexing_conveyor
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::register)
install(TARGETS hmlv_indexing_conveyor LIBRARY DESTINATION lib)

install(DIRECTORY launch config robots DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/transfer_scene.py scripts/transfer_demo.py scripts/record_run.py
        scripts/reach_sweep.py scripts/duo_scene.py scripts/duo_demo.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
