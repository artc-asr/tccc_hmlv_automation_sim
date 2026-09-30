cmake_minimum_required(VERSION 3.8)
project(moz1_sim_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)

# Rigid sim-only grasp (see src/kinematic_grasp.cc); sim_gazebo.launch.py puts
# this lib dir on IGN_GAZEBO_SYSTEM_PLUGIN_PATH.
add_library(moz1_kinematic_grasp SHARED src/kinematic_grasp.cc)
target_compile_features(moz1_kinematic_grasp PRIVATE cxx_std_17)
target_link_libraries(moz1_kinematic_grasp
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::register)
install(TARGETS moz1_kinematic_grasp LIBRARY DESTINATION lib)

# Conveyor with a filling station (see src/conveyor_belt.cc), a world plugin.
add_library(moz1_conveyor_belt SHARED src/conveyor_belt.cc)
target_compile_features(moz1_conveyor_belt PRIVATE cxx_std_17)
target_link_libraries(moz1_conveyor_belt
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::register)
install(TARGETS moz1_conveyor_belt LIBRARY DESTINATION lib)

# gz model poses -> ROS TF by model name, so RViz can show moving objects
# (see src/gz_pose_tf.cc).
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
add_executable(gz_pose_tf src/gz_pose_tf.cc)
target_compile_features(gz_pose_tf PRIVATE cxx_std_17)
target_link_libraries(gz_pose_tf
  ignition-transport11::ignition-transport11
  ignition-msgs8::ignition-msgs8)
ament_target_dependencies(gz_pose_tf rclcpp tf2_ros geometry_msgs)
install(TARGETS gz_pose_tf RUNTIME DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/scene_markers.py scripts/grasp_helper.py
           scripts/jerrycan_scene.py scripts/jerrycan_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
