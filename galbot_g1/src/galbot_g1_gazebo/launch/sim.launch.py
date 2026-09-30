"""Galbot G1 in Gazebo Harmonic with ros2_control, holonomic base and sensors.

Gazebo runs as a headless server; RViz is the viewer (robot, TF, sensors and the world's
models mirrored as markers by world_markers.py). gui:=true also opens Gazebo's own GUI.

    ros2 launch galbot_g1_gazebo sim.launch.py                  # headless physics + RViz
    ros2 launch galbot_g1_gazebo sim.launch.py gui:=true     # also the Gazebo GUI window
    ros2 launch galbot_g1_gazebo sim.launch.py rviz:=false      # no viewer at all
"""

import json
import os
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# base_footprint sits 0.028166 m below base_link.
BASE_LINK_HEIGHT = '0.028166'

# RGB-D camera -> body frame (x forward, z up) in which Gazebo expresses its point cloud.
CAMERAS = {
    'head_camera': 'head_camera_link',
    'left_wrist_camera': 'left_wrist_camera_sensor_link',
    'right_wrist_camera': 'right_wrist_camera_sensor_link',
}

CONTROLLERS = [
    'joint_state_broadcaster',
    'leg_controller',
    'head_controller',
    'left_arm_controller',
    'right_arm_controller',
    'left_gripper_controller',
    'right_gripper_controller',
]


def use_mjcf_actuator_limits(urdf, description_share):
    """Raise joint effort limits to the actuator force ranges of Galbot's MuJoCo model.

    The URDF effort limits are too low for Gazebo's joint servos to hold the leg up
    against the upper body's weight; the upstream MJCF config carries the force ranges
    Galbot uses in simulation. Gripper joints keep their (already higher) URDF limits.
    """
    with open(os.path.join(description_share, 'config', 'mjcf', 'joint_data.json')) as f:
        joint_data = json.load(f)['joints']

    robot = ET.fromstring(urdf)
    for joint in robot.iter('joint'):
        actuator = joint_data.get(joint.get('name'), {}).get('actuator', {})
        limit = joint.find('limit')
        if 'forcerange' not in actuator or limit is None:
            continue
        effort = max(float(limit.get('effort', 0.0)), max(abs(v) for v in actuator['forcerange']))
        limit.set('effort', str(effort))
    return ET.tostring(robot, encoding='unicode')


def robot_state_publisher(context, pkg_share, description_share):
    urdf = xacro.process_file(
        os.path.join(pkg_share, 'urdf', 'galbot_g1.gazebo.xacro'),
        mappings={'enable_sensors': LaunchConfiguration('sensors').perform(context)},
    ).toxml()
    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': use_mjcf_actuator_limits(urdf, description_share),
                     'use_sim_time': True}],
        output='screen')]


def generate_launch_description():
    pkg_share = get_package_share_directory('galbot_g1_gazebo')
    description_share = get_package_share_directory('galbot_one_golf_description')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    sensors = LaunchConfiguration('sensors')

    gz_sim_launch = PythonLaunchDescriptionSource(PathJoinSubstitution(
        [get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py']))

    # Server always; the GUI is a separate client process, only with gui:=true.
    gazebo_server = IncludeLaunchDescription(
        gz_sim_launch,
        launch_arguments={'gz_args': [world, ' -s -r -v 3 --headless-rendering'],
                          'on_exit_shutdown': 'true'}.items())

    gazebo_gui = IncludeLaunchDescription(
        gz_sim_launch,
        launch_arguments={'gz_args': '-g -v 3'}.items(),
        condition=IfCondition(gui))

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'galbot_g1', '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', BASE_LINK_HEIGHT, '-Y', LaunchConfiguration('yaw')],
        output='screen')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': os.path.join(pkg_share, 'config', 'bridge.yaml'),
                     'use_sim_time': True}],
        output='screen')

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[f'/{cam}/{stream}' for cam in CAMERAS for stream in ('image', 'depth_image')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(sensors),
        output='screen')

    point_cloud_relays = [
        Node(
            package='galbot_g1_gazebo',
            executable='pointcloud_frame_relay',
            name=f'{cam}_points_relay',
            parameters=[{'frame_id': body_frame, 'use_sim_time': True}],
            remappings=[('input', f'/{cam}/points_raw'), ('output', f'/{cam}/points')],
            condition=IfCondition(sensors),
            output='screen')
        for cam, body_frame in CAMERAS.items()
    ]

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=CONTROLLERS + ['--controller-manager', '/controller_manager',
                                 '--controller-manager-timeout', '120'],
        output='screen')

    # RViz can't see inside gz-sim: mirror the world's models as markers.
    world_markers = Node(
        package='galbot_g1_gazebo',
        executable='world_markers.py',
        parameters=[{'world_file': world, 'frame_id': 'odom', 'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'galbot_g1.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('world',
                              default_value=os.path.join(pkg_share, 'worlds', 'galbot_table.sdf'),
                              description='Gazebo world SDF file'),
        DeclareLaunchArgument('gui', default_value='false',
                              description="Also open Gazebo's own GUI (RViz is the default viewer)"),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz (+ world markers), the simulation viewer'),
        DeclareLaunchArgument('sensors', default_value='true',
                              description='Simulate lidar and RGB-D cameras'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        # Lets Gazebo resolve package://galbot_one_golf_description/... mesh URIs.
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(description_share)),

        gazebo_server,
        gazebo_gui,
        OpaqueFunction(function=robot_state_publisher, args=[pkg_share, description_share]),
        spawn,
        bridge,
        image_bridge,
        *point_cloud_relays,
        RegisterEventHandler(OnProcessExit(target_action=spawn,
                                           on_exit=[controller_spawner])),
        world_markers,
        rviz_node,
    ])
