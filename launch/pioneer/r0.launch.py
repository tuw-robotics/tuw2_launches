import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time     = LaunchConfiguration('use_sim_time',  default='false')
    namespace_arg    = DeclareLaunchArgument('namespace',   default_value=TextSubstitution(text=''))
    model_name_arg   = DeclareLaunchArgument('model_name',  default_value=TextSubstitution(text='robot0'))
    robot_arg        = DeclareLaunchArgument('robot',       default_value=TextSubstitution(text='pioneer3dx'))
    
    this_pgk = 'tuw2_launches'
    config_pgk_dir = get_package_share_directory(this_pgk)
    
    return LaunchDescription([
        namespace_arg,
        model_name_arg,
        robot_arg,
        IncludeLaunchDescription( PythonLaunchDescriptionSource( os.path.join(config_pgk_dir, 'launch', 'pioneer', 'description.launch.py'))),
        IncludeLaunchDescription( PythonLaunchDescriptionSource( os.path.join(config_pgk_dir, 'launch', 'pioneer', 'hokuyo.launch.py')))
    ])


