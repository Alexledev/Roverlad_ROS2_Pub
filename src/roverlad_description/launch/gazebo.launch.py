import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    roverlad_description = get_package_share_directory("roverlad_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        roverlad_description, "urdf", "roverlad.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(roverlad_description).parent.resolve())
            ]
        )
    
    ros_distro = os.environ["ROS_DISTRO"]
    
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    world_path = os.path.join(roverlad_description, "worlds", "default.sdf")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
            ), "/gz_sim.launch.py"
        ]),
        launch_arguments=[
            ("gz_args", [f"-v 4 -r {world_path}"]) # --physics-engine gz-physics-dartsim-plugin
        ],
    )
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "roverlad"
                   "-z", "0.05",],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ]
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])