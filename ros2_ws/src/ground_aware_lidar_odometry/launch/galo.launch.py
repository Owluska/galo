from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
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
    declare_launch_static_tf = DeclareLaunchArgument(
        'launch_static_tf',
        default_value='false',
        description='Launch static_tf_publisher/static_tf.launch.py if true'
    )

    # Path to the main configuration file. Load the parameter dictionary
    # directly so all GALO nodes receive the same YAML values.
    params_file = os.path.join(
        get_package_share_directory("ground_aware_lidar_odometry"),
        "config",
        "galo_params.yaml",
    )
    with open(params_file, "r") as f:
        galo_params = yaml.safe_load(f)["galo"]["ros__parameters"]

    # Get the value of use_sim_time as a LaunchConfiguration
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_static_tf = LaunchConfiguration('launch_static_tf')

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("static_tf_publisher"),
                "launch",
                "static_tf.launch.py",
            )
        ),
        condition=IfCondition(launch_static_tf),
    )

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

    frontend_node = Node(
        package="ground_aware_lidar_odometry",
        executable="frontend_node",
        name="galo_frontend",
        output="screen",
        parameters=[
            galo_params,
            {"use_sim_time": use_sim_time}
        ]
    )

    # Odometry backend consumes /GALO/frame_features from galo_frontend.
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
        declare_launch_static_tf,
        log_info,
        static_tf_launch,
        deskew_node,
        frontend_node,
        galo_node,
    ])


# Default (use_sim_time = false)
# ros2 launch ground_aware_lidar_odometry your_launch_file.py

# Override to true
# ros2 launch ground_aware_lidar_odometry your_launch_file.py use_sim_time:=true
