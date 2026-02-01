import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy

def generate_launch_description():
    rviz_config_dir = os.path.join(
        get_package_share_directory('roverlad_bringup'), 'rviz', 'rove_around.rviz'
    )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_dir]
    # )

    twist_mux_config = os.path.join(
        get_package_share_directory('roverlad_bringup'),
        'config',
        'twist_mux.yaml'
    )

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roverlad_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roverlad_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )

    # Run rover_comms node
    rover_comms_node = Node(
        package="roverlad_firmware",
        executable="rover_comms",
        name="rover_comms",
        output="screen",
        parameters=[{"port": "/dev/ttyACM0"}]
    )

    ble_reader_node = Node(
        package="ble_reader_py",
        executable="ble_node",
    )

    ble_cmd_node = Node(
        package="roverlad_firmware",
        executable="controller_teleop",
        name="controller_teleop"
    )

    mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        remappings=[
            ('/cmd_vel_out', '/cmd_vel_presafe'),
        ],
        parameters=[twist_mux_config],
        output='screen'
    )

    safety_node = Node(
        package="roverlad_safety",
        executable="safety_node.py",
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/roverlad_controller/cmd_vel'),
        ],
    )

    # imu_driver_node = Node(
    #     package="roverlad_firmware",
    #     executable="mpu6050_driver.py"
    # )
    
    return LaunchDescription([
        # rviz_node,
        hardware_interface,
        controller,
        rover_comms_node,
        # ble_reader_node,
        # ble_cmd_node,
        # rover_cam,
        mux,
        twist_stamper,
        safety_node
        # joystick,
        # imu_driver_node,
    ])