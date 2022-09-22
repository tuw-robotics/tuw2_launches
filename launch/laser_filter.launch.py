from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("tuw2_launches"),
                    "config/laser_filter/p3dx", "shadow_filter_example.yaml",
                ])],
            remappings=[
                ("scan", "scan_raw"),
                ("scan_filtered", "scan"),]
        )
    ])
