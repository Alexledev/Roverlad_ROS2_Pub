from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os


# def log_message(context, *args, **kwargs):
#     # The log file path
#     log_file = os.path.join(
#         "/home/jetsonalex", 
#         "ros2_logs", 
#         "launch_log.txt"
#     )
#     os.makedirs(os.path.dirname(log_file), exist_ok=True)
#     with open(log_file, "a") as f:
#         f.write("mapping bringup executed!.\n")
#     return []


def generate_launch_description():


    roverlad = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('roverlad_bringup'),
                'launch',
                'roverlad_real.launch.py'
            )
        )
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'),
                'launch',
                'ydlidar_launch.py'
            )
        )
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('roverlad_localization'),
                'launch',
                'global_localization.launch.py'
            )
        )
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('roverlad_mapping'),
                'launch',
                'slam.launch.py'
            )
        )
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('roverlad_navigation'),
                'launch',
                'navigation.launch.py'
            )
        )
    )


    config_server = Node(
        package='roverlad_navigation',
        executable='config_node_server.py',
        name='config_node_server',
    )

    slam_manage_server = Node(
        package='roverlad_mapping',
        executable='slam_manage_server.py',
        name='slam_manage_server',
    )

    map_getter_server = Node(
        package='roverlad_mapping',
        executable='map_getter_server.py',
        name='map_getter_server',
    )


    return LaunchDescription([
        roverlad,
        lidar,
        localization,
        slam,
        TimerAction(period=6.0, actions=[navigation]),   # t = 15s,
        TimerAction(period=8.0, actions=[config_server]),
        TimerAction(period=8.0, actions=[slam_manage_server]),
        TimerAction(period=8.0, actions=[map_getter_server])
    ])
