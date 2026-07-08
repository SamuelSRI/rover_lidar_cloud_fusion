from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("rover_lidar_cloud_fusion")

    config_file = os.path.join(
        pkg_share,
        "config",
        "lidar_cloud_fusion.yaml"
    )

    return LaunchDescription([
        Node(
            package="rover_lidar_cloud_fusion",
            executable="lidar_cloud_fusion_node",
            name="lidar_cloud_fusion_node",
            output="screen",
            parameters=[config_file],
        )
    ])
