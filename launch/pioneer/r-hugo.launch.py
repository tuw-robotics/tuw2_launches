import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    tuw_dir = get_package_share_directory('tuw_bringup')
    aria_dir = get_package_share_directory('pioneer_aria')

    aria_params = os.path.join(tuw_dir, 'config', 'pioneer', 'aria.yaml')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    with_lidar = LaunchConfiguration('with_lidar')
    serial_port = LaunchConfiguration('serial_port')

    # Rewrite serial_port in-memory before the node loads the file.
    # root_key='' because the YAML uses a fixed top-level key "aria",
    # not the namespace. See note below if you namespace the node.
    configured_params = RewrittenYaml(
        source_file=aria_params,
        root_key='',
        param_rewrites={
            'serial_port': serial_port,
        },
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('with_lidar', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyS0'),



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
                'params_file': configured_params,
            }.items()),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(tuw_dir, 'launch', 'pioneer', 'lidar.launch.py')),
            launch_arguments={
                'port_name': LaunchConfiguration('lidar_port'),
            }.items(),
            condition=IfCondition(with_lidar)),
    ])
