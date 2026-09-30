# Convert the Livox MID360 PointCloud2 into a 2D LaserScan for slam_toolbox / Nav2.
#
# Input topic depends on the Livox driver xfer_format:
#   xfer_format=0  -> /livox/lidar          (PointCloud2)   <-- needed here
#   xfer_format=1  -> /livox/lidar          (livox custom)  <-- NOT compatible
#   xfer_format=2  -> /livox/points         (PCL)
# So when running this launch file, also pass xfer_format:=0 to livox_bringup.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('moz1_navigation_bringup')

    cloud_in = LaunchConfiguration('cloud_in')
    scan_out = LaunchConfiguration('scan_out')
    params_file = LaunchConfiguration('pc_to_scan_params')

    default_params = PathJoinSubstitution(
        [pkg_share, 'config', 'pointcloud_to_scan.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'cloud_in', default_value='/livox/lidar',
            description='Input PointCloud2 topic from the Livox driver.'),
        DeclareLaunchArgument(
            'scan_out', default_value='/scan',
            description='Output LaserScan topic.'),
        # Uniquely named to avoid ROS2 launch's quirk where LaunchConfiguration
        # 'params_file' is global across IncludeLaunchDescription children.
        DeclareLaunchArgument(
            'pc_to_scan_params', default_value=default_params,
            description='YAML with pointcloud_to_laserscan params.'),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', cloud_in),
                ('scan', scan_out),
            ],
            parameters=[params_file],
            output='screen',
        ),
    ])
