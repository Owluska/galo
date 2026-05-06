from launch.actions import LogInfo
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare("ground_aware_lidar_odometry"),
        "config",
        "galo_params.yaml",
    ])

    return LaunchDescription([
        LogInfo(msg=["Using GALO params file: ", params_file]),

        Node(
            package="ground_aware_lidar_odometry",
            executable="node",
            name="galo",
            output="screen",
            parameters=[params_file],
        )
    ])