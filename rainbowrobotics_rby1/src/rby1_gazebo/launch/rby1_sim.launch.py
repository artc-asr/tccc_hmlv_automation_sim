"""Launch Gazebo Harmonic with the RB-Y1 (Model A v1.2) and its ros2_control controllers.

Default: Gazebo runs headless (server only) and RViz is the viewer; the world's
objects are drawn in RViz by world_markers.py.

    ros2 launch rby1_gazebo rby1_sim.launch.py                       # headless Gazebo + RViz
    ros2 launch rby1_gazebo rby1_sim.launch.py gui:=true rviz:=false # Gazebo GUI only
"""
import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

CONTROLLERS = [
    'diff_drive_controller',
    'torso_controller',
    'right_arm_controller',
    'left_arm_controller',
    'head_controller',
    'right_gripper_controller',
    'left_gripper_controller',
]


def generate_launch_description():
    gz_pkg = FindPackageShare('rby1_gazebo')
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('rviz')
    # gui:=false runs the server only (sensors still render offscreen).
    headless_flags = PythonExpression(
        ["'' if '", LaunchConfiguration('gui'), "'.lower() == 'true' else '-s --headless-rendering '"])

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([gz_pkg, 'urdf', 'rby1_gazebo.urdf.xacro'])]),
        value_type=str)

    # Let Gazebo resolve package://rby1_description/... mesh URIs.
    description_share_parent = os.path.join(get_package_prefix('rby1_description'), 'share')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ['-r -v 3 ', headless_flags, world], 'on_exit_shutdown': 'true'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'rby1',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': PathJoinSubstitution([gz_pkg, 'config', 'gz_bridge.yaml']),
                     'use_sim_time': True}])

    jsb_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'])

    controllers_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[*CONTROLLERS, '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'])

    # Gazebo world frame -> odom (odom starts at the spawn pose).
    world_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher', name='world_to_odom',
        arguments=['--x', LaunchConfiguration('x'), '--y', LaunchConfiguration('y'),
                   '--yaw', LaunchConfiguration('yaw'), '--frame-id', 'world', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}])

    world_markers = Node(
        package='rby1_gazebo', executable='world_markers.py', condition=IfCondition(use_rviz),
        parameters=[{'world_file': world, 'frame_id': 'world'}])

    rviz = Node(
        package='rviz2', executable='rviz2', condition=IfCondition(use_rviz),
        arguments=['-d', PathJoinSubstitution([gz_pkg, 'config', 'rby1_sim.rviz'])],
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution([gz_pkg, 'worlds', 'rby1_world.sdf'])),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz as the viewer'),
        DeclareLaunchArgument('gui', default_value='false', description='Also show the Gazebo GUI'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.01'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', description_share_parent),
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[controllers_spawner])),
        world_to_odom,
        world_markers,
        rviz,
    ])
