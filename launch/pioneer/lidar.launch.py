import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    this_pgk = 'tuw_bringup'
    config_pgk_dir = get_package_share_directory(this_pgk)
    return LaunchDescription([
        DeclareLaunchArgument('port_name', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('frame_id', default_value='laser0'),
        DeclareLaunchArgument('topic_name', default_value='scan_raw'),

        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='ldlidar',
            output='screen',
            parameters=[{
                'product_name': 'LDLiDAR_STL27L',
                'topic_name': LaunchConfiguration('topic_name'),
                'frame_id': LaunchConfiguration('frame_id'),
                'port_name': LaunchConfiguration('port_name'),
                'port_baudrate': 921600,
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
                'angle_crop_min': 0.0,
                'angle_crop_max': 0.0,
            }],
        ),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pgk_dir, 'launch', 'laser_filter.launch.py'))
        )
    ])
