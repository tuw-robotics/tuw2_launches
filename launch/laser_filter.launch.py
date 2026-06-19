from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    this_pgk = 'tuw_bringup'
    this_pgk_dir = get_package_share_directory(this_pgk)
    return LaunchDescription([
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                PathJoinSubstitution([
                    this_pgk_dir, "config/laser_filter/pioneer2dx", "shadow_filter_example.yaml",
                ])],
            remappings=[
                ("scan", "scan_raw"),
                ("scan_filtered", "scan"),]
        )
    ])
