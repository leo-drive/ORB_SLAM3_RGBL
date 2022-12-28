
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orb_slam3_mapper',
            executable='mapper_node',
            name='mapper_node',
            output="screen",
            parameters=[
                os.path.join(
                    get_package_share_directory("orb_slam3_mapper"),
                    "config",
                    "config.yaml",
                )
            ]
        )
    ])