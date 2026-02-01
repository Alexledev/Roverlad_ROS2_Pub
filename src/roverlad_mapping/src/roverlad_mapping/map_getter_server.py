#!/usr/bin/env python3

import asyncio
from pathlib import Path
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

from roverlad_interfaces.action import SwitchMap


class MapGetterServer(Node):
    def __init__(self):
        super().__init__('map_getter_server')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.enableMapServices = self.create_subscription(String, 'roverlad_server/enable_map_get', self.init_map_services, qos)
        self.switchMap = ActionServer(self, SwitchMap, "roverlad_server/switch_map", execute_callback=self.execute_switch_map_cb,
                                                                                         goal_callback=self.goal_switch_map_cb,
                                                                                     cancel_callback=self.cancel_switch_map_cb,)

        self.cli_mpsr = None
        self.cli_amcl = None


        pkg_share = get_package_share_directory("roverlad_mapping")
        self.map_general_path = Path(pkg_share) / "maps" #
        
        self.setMapClient = self.create_client(SetParameters, '/map_server/set_parameters')

        while not self.setMapClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for map_server parameter service...')

        
        self.get_logger().info('map_getter_server started')


    def init_map_services(self, msg):
        self.cli_mpsr = self.create_client(ChangeState, '/map_server/change_state')
        self.cli_amcl = self.create_client(ChangeState, '/amcl/change_state')

        while not self.cli_mpsr.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /map_server/change_state...')

        while not self.cli_amcl.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /amcl/change_state...')


        self.init_map_server()
        # self.init_amcl()

    def transition_state(self, cli, transition_id: int):
        req = ChangeState.Request()
        req.transition.id = transition_id
        return cli.call_async(req)


    def init_map_server(self):
        fut1 = self.transition_state(self.cli_mpsr, Transition.TRANSITION_CONFIGURE)
        fut1.add_done_callback(self.on_map_server_configured)

    def on_map_server_configured(self, _):
        fut2 = self.transition_state(self.cli_mpsr, Transition.TRANSITION_ACTIVATE)
        fut2.add_done_callback(self.on_map_server_activated)
    
    def on_map_server_activated(self, _):
        self.get_logger().info('map server initialized')


    def init_amcl(self):
        fut1 = self.transition_state(self.cli_amcl, Transition.TRANSITION_CONFIGURE)
        fut1.add_done_callback(self.on_amcl_configured)

    def on_amcl_configured(self, _):
        fut2 = self.transition_state(self.cli_amcl, Transition.TRANSITION_ACTIVATE)
        fut2.add_done_callback(self.on_amcl_activated)
    
    def on_amcl_activated(self, _):
        self.get_logger().info('amcl initialized')



    def reset_map_server(self):
        fut1 = self.transition_state(self.cli_mpsr, Transition.TRANSITION_DEACTIVATE)
        fut1.add_done_callback(self.on_map_server_deactivate)

    def on_map_server_deactivate(self, _):
        fut2 = self.transition_state(self.cli_mpsr, Transition.TRANSITION_CLEANUP)
        fut2.add_done_callback(self.on_map_server_cleanup)
    
    def on_map_server_cleanup(self, _):
        self.get_logger().info('map server deactivated')


    def goal_switch_map_cb(self, goal_request):
        self.get_logger().info(f'>Attempt Switch map to {goal_request.map_name}')
        
        return GoalResponse.ACCEPT

    def cancel_switch_map_cb(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    async def execute_switch_map_cb(self, goal_handle):
        # self.transition(self.cli_mpsr, Transition.TRANSITION_DEACTIVATE)
        # self.transition(self.cli_mpsr, Transition.TRANSITION_CLEANUP)
        result = SwitchMap.Result()
        result.success = True

        self.reset_map_server()
        self.get_logger().info(f"Switch to {goal_handle.request.map_name}")

        fullPath = self.map_general_path / goal_handle.request.map_name

        param = Parameter(name='yaml_filename', value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=str(fullPath)))

        req = SetParameters.Request()
        req.parameters = [param]

        future = self.setMapClient.call_async(req)
        await future
        self.get_logger().info("Switched maps -> begin reinit")


        self.init_map_server()
        self.init_amcl()

        # await asyncio.sleep(2.0)

        goal_handle.succeed()
        return result

        # self.transition(self.cli_mpsr, Transition.TRANSITION_CONFIGURE)
        # self.transition(self.cli_mpsr, Transition.TRANSITION_ACTIVATE)

        # self.transition(self.cli_amcl, Transition.TRANSITION_DEACTIVATE)
        # self.transition(self.cli_amcl, Transition.TRANSITION_ACTIVATE)

def main():
    rclpy.init()
    node = MapGetterServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()