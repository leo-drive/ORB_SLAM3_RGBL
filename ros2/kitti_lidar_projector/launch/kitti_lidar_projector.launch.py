
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kitti_lidar_projector',
            executable='kitti_lidar_projector',
            name='kitti_lidar_projector',
            output="screen",
            parameters=[
                os.path.join(
                    get_package_share_directory("kitti_lidar_projector"),
                    "config",
                    "config.yaml",
                )
            ]
        )
    ])