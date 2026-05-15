from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    params_file = os.path.join(
        get_package_share_directory("ground_aware_lidar_odometry"),
        "config",
        "galo_params.yaml",
    )
    with open(params_file, "r") as f:
        galo_params = yaml.safe_load(f)["galo"]["ros__parameters"]

    use_sim_time = LaunchConfiguration('use_sim_time')
    parameters = [galo_params, {"use_sim_time": use_sim_time}]

    container = ComposableNodeContainer(
        name="galo_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="ground_aware_lidar_odometry",
                plugin="ground_aware_lidar_odometry::GaloDeskewComponent",
                name="galo_deskew",
                parameters=parameters,
            ),
            ComposableNode(
                package="ground_aware_lidar_odometry",
                plugin="ground_aware_lidar_odometry::GaloFrontendComponent",
                name="galo_frontend",
                parameters=parameters,
            ),
            ComposableNode(
                package="ground_aware_lidar_odometry",
                plugin="GaloOdometryComponent",
                name="galo",
                parameters=parameters,
            ),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        LogInfo(msg=["Using GALO params file: ", params_file]),
        container,
    ])
