#!/usr/bin/env python3

import asyncio
from pathlib import Path
import subprocess
from ament_index_python import get_package_share_directory
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from std_msgs.msg import String

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from roverlad_interfaces.action import ResetRos


class CommandsServer(Node):
    def __init__(self):
        super().__init__('commands_server')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.resetRos = ActionServer(self, ResetRos, "roverlad_server/reset_ros", execute_callback=self.execute_reset_ros_cb,
                                                                                         goal_callback=self.goal_reset_ros_cb,
                                                                                     cancel_callback=self.cancel_reset_ros_cb,)   
        
        self.get_logger().info('commands_server started')

    def goal_reset_ros_cb(self, goal_request):
        self.get_logger().info(f'>Attempt Reset Ros...')
        
        return GoalResponse.ACCEPT

    def cancel_reset_ros_cb(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    async def execute_reset_ros_cb(self, goal_handle):
        result = ResetRos.Result()

        # Reset
        try:

            self.get_logger().info('Begin Restart')
            password = goal_handle.request.password
            proc = subprocess.run(["sudo", "-S", "systemctl", "restart", "ros2-startup.service"], input=f"{password}\n", text=True, check=True)
            
            self.get_logger().info('Finish restart process')
            result.success = True
            goal_handle.succeed()
        except Exception as e:
            self.get_logger().error(f'>Something went wrong while resetting {e}')
            goal_handle.abort()
        finally:
            return result

def main():
    rclpy.init()
    node = CommandsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()