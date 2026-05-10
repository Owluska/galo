from launch import LaunchDescription
from launch.actions import LogInfo
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare the use_sim_time argument with a default value
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # Path to the main configuration file
    params_file = PathJoinSubstitution([
        FindPackageShare("ground_aware_lidar_odometry"),
        "config",
        "galo_params.yaml",
    ])

    # Get the value of use_sim_time as a LaunchConfiguration
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Log the parameter file path (will be resolved at runtime)
    log_info = LogInfo(msg=["Using GALO params file: ", params_file])

    # Node with both the YAML parameters and the use_sim_time override
    galo_node = Node(
        package="ground_aware_lidar_odometry",
        executable="node",
        name="galo",
        output="screen",
        parameters=[
            params_file,                # load everything from YAML
            {"use_sim_time": use_sim_time}  # override use_sim_time
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        log_info,
        galo_node,
    ])


# Default (use_sim_time = false)
# ros2 launch ground_aware_lidar_odometry your_launch_file.py

# Override to true
# ros2 launch ground_aware_lidar_odometry your_launch_file.py use_sim_time:=true