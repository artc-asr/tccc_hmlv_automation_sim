# Nav2 planning + control stack (no localization).
#
# Wraps nav2_bringup's navigation_launch.py and feeds it our params. Use this
# WITH slam_toolbox running (slam_toolbox publishes map -> odom). If you want
# to localize against a saved map instead, swap to nav2_bringup's
# localization_launch.py + bringup_launch.py.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('moz1_navigation_bringup')
    nav2_bringup_share = FindPackageShare('nav2_bringup')

    params_file = LaunchConfiguration('nav2_params')
    use_composition = LaunchConfiguration('use_composition')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    default_params = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])

    nav2_launch = PathJoinSubstitution(
        [nav2_bringup_share, 'launch', 'navigation_launch.py']
    )

    return LaunchDescription([
        # Uniquely named to avoid clashing with other children of
        # full_nav_bringup that also declare params_file.
        DeclareLaunchArgument(
            'nav2_params', default_value=default_params,
            description='Nav2 servers parameter YAML.'),
        DeclareLaunchArgument(
            'use_composition', default_value='False',
            description='Whether to run Nav2 as composable nodes (faster) or '
                        'separate processes (easier to debug). Default False.'),
        DeclareLaunchArgument(
            'autostart', default_value='True',
            description='Auto-activate the lifecycle stack on launch.'),
        DeclareLaunchArgument(
            'log_level', default_value='info'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch]),
            launch_arguments={
                'params_file': params_file,
                'use_composition': use_composition,
                'autostart': autostart,
                'log_level': log_level,
                'use_sim_time': 'False',
            }.items(),
        ),
    ])
