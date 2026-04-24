from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "static_tf_publisher"

    config_file = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "sensor_transforms.yaml"
    )

    return LaunchDescription([
        Node(
            package=package_name,
            executable="static_tf_publisher",
            name="static_tf_publisher",
            output="screen",
            parameters=[config_file]
        )
    ])