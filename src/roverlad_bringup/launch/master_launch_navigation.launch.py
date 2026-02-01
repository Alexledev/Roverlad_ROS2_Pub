from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def ensure_configured(node):
    return ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'ros2 lifecycle get {node} | grep -q unconfigured && '
            f'ros2 lifecycle set {node} configure || true'
        ],
        output='screen'
    )

def ensure_active(node):
    return ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'ros2 lifecycle get {node} | grep -q inactive && '
            f'ros2 lifecycle set {node} activate || true'
        ],
        output='screen'
    )

def ensure_nav2_ready(nodes):
    actions = []
    for n in nodes:
        actions.append(ensure_configured(n))
    for n in nodes:
        actions.append(ensure_active(n))
    return actions

def generate_launch_description():

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('roverlad_bringup'),
                'launch',
                'roverlad_real.launch.py'
            )
        )
    )

    ydlidar = IncludeLaunchDescription(
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

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('roverlad_navigation'),
                'launch',
                'navigation.launch.py'
            )
        )
    )

    # set_initial_pose = ExecuteProcess(
    #     cmd=[
    #         'ros2', 'topic', 'pub', '-1',
    #         '/initialpose',
    #         'geometry_msgs/msg/PoseWithCovarianceStamped',
    #         "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
    #     ],
    #     output='screen'
    # )

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

    NAV2_LIFECYCLE_NODES = [
        '/map_server',
        '/planner_server',
        '/controller_server',
        '/smoother_server',
        '/behavior_server',
        '/bt_navigator',
    ]


    return LaunchDescription([
        TimerAction(period=0.0, actions=[bringup]),        
        TimerAction(period=1.0, actions=[ydlidar]),        
        TimerAction(period=2.0, actions=[localization]), 
        TimerAction(period=3.0, actions=[navigation]),   
        TimerAction(
            period=8.0,
            actions=ensure_nav2_ready(NAV2_LIFECYCLE_NODES)
        ),
        TimerAction(period=8.0, actions=[config_server]),
        TimerAction(period=8.0, actions=[slam_manage_server]),
        TimerAction(period=8.0, actions=[map_getter_server])


                                         
        # TimerAction(period=10.0, actions=[server_rover]),  
        # TimerAction(period=10.0, actions=[server_webcam]),  
    ])
