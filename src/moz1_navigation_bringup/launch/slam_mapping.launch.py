# slam_toolbox in online async mapping mode.
#
# Inputs (must already be running before this launch):
#   /scan        from pointcloud_to_scan.launch.py
#   odom -> base_link TF  from odom_ekf.launch.py (or odom_publisher.launch.py)
#
# Output:
#   /map         occupancy grid (saved later via nav2_map_server map_saver_cli)
#   map -> odom  TF (slam_toolbox publishes this; do NOT also run a static one)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('moz1_navigation_bringup')

    params_file = LaunchConfiguration('slam_params')
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_params = PathJoinSubstitution([pkg_share, 'config', 'slam_toolbox.yaml'])

    return LaunchDescription([
        # Uniquely named to avoid ROS2 launch's quirk where LaunchConfiguration
        # 'params_file' is global across IncludeLaunchDescription children.
        DeclareLaunchArgument(
            'slam_params', default_value=default_params,
            description='slam_toolbox YAML params.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false'),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])
