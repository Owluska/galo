from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ground_aware_lidar_odometry',
            executable='node',
            name='GALOnode',
            output='screen'
        )
    ])