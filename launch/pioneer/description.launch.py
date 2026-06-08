import os

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('tuw2_launches')
    xacro_file = os.path.join(pkg, 'config', 'robot_description', 'pioneer2dx', 'main.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='robot_state_publisher',
            namespace=namespace,
            executable='robot_state_publisher',
            name='state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_frequency': 30.0,
                'robot_description': robot_description,
            }],
            remappings=remappings,
        ),
        Node(
            package='joint_state_publisher',
            namespace=namespace,
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
            remappings=remappings,
        ),
    ])
