from launch import LaunchDescription
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Declare the use_sim_time argument with a default value
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # Path to the main configuration file. Load the parameter dictionary
    # directly so both galo and galo_deskew receive the same YAML values.
    params_file = os.path.join(
        get_package_share_directory("ground_aware_lidar_odometry"),
        "config",
        "galo_params.yaml",
    )
    with open(params_file, "r") as f:
        galo_params = yaml.safe_load(f)["galo"]["ros__parameters"]

    # Get the value of use_sim_time as a LaunchConfiguration
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Log the parameter file path (will be resolved at runtime)
    log_info = LogInfo(msg=["Using GALO params file: ", params_file])

    deskew_node = Node(
        package="ground_aware_lidar_odometry",
        executable="deskew_node",
        name="galo_deskew",
        output="screen",
        parameters=[
            galo_params,
            {"use_sim_time": use_sim_time}
        ]
    )

    # Odometry backend consumes /GALO/deskewed_cloud from galo_deskew.
    galo_node = Node(
        package="ground_aware_lidar_odometry",
        executable="node",
        name="galo",
        output="screen",
        parameters=[
            galo_params,
            {"use_sim_time": use_sim_time}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        log_info,
        deskew_node,
        galo_node,
    ])


# Default (use_sim_time = false)
# ros2 launch ground_aware_lidar_odometry your_launch_file.py

# Override to true
# ros2 launch ground_aware_lidar_odometry your_launch_file.py use_sim_time:=true
