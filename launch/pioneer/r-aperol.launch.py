import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tuw_dir = get_package_share_directory('tuw2_launches')
    aria_dir = get_package_share_directory('pioneer_aria')

    aria_params = os.path.join(tuw_dir, 'config', 'pioneer', 'aria.yaml')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    with_lidar = LaunchConfiguration('with_lidar')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('with_lidar', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(tuw_dir, 'launch', 'pioneer', 'description.launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
            }.items()),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(aria_dir, 'launch', 'aria.launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'params_file': aria_params,
            }.items()),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(tuw_dir, 'launch', 'pioneer', 'lidar.launch.py')),
            launch_arguments={
                'port_name': LaunchConfiguration('lidar_port'),
            }.items(),
            condition=IfCondition(with_lidar)),
    ])
