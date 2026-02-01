import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def log_message(context, *args, **kwargs):
    # The log file path
    log_file = os.path.join(
        "/home/jetsonalex", 
        "ros2_logs", 
        "slam_log.txt"
    )
    os.makedirs(os.path.dirname(log_file), exist_ok=True)
    with open(log_file, "a") as f:
        f.write("SLAM executed!.\n")
    return []

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_config = LaunchConfiguration("slam_config")

    # ros_distro = os.environ["ROS_DISTRO"]
    lifecycle_nodes = ["map_saver_server"]
    # if ros_distro != "humble":
    #     lifecycle_nodes.append("slam_toolbox")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("roverlad_mapping"),
            "config",
            "slam_toolbox.yaml"
        ),
        description="Full path to slam yaml file to load"
    )
    
    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        # parameters=[
        #     {"save_map_timeout": 5.0},
        #     {"use_sim_time": use_sim_time},
        #     {"free_thresh_default": "0.196"},
        #     {"occupied_thresh_default": "0.65"},
        #     {"map_subscribe_transient_local": "true"}
        # ],
        # respawn=True,
        # respawn_delay=0.5,
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_config_arg,
        OpaqueFunction(function=log_message),
        nav2_map_saver,
        slam_toolbox,
        nav2_lifecycle_manager,
    ])