import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    this_pgk = 'tuw2_launches'
    config_pgk_dir = get_package_share_directory(this_pgk)
    config_laser = os.path.join(config_pgk_dir, "config", "pioneer", "hokuyo", "urg_node_ethernet.yaml")
    def expand_param_file_name(context):
        if os.path.exists(config_laser):
            print("hihi")
            return [SetLaunchConfiguration('param', config_laser)]

    param_file_path = OpaqueFunction(function=expand_param_file_name)

    hokuyo_node = Node(
        package='urg_node', 
        executable='urg_node_driver', 
        output='screen',     
        remappings=[("scan", "scan_raw")],
        parameters=[LaunchConfiguration('param')]   
    )

    laser_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pgk_dir, 'launch', 'laser_filter.launch.py'))
    )

    launch_description = LaunchDescription()
    launch_description.add_action(laser_filter_launch)
    launch_description.add_action(param_file_path)
    launch_description.add_action(hokuyo_node)
    return launch_description 

