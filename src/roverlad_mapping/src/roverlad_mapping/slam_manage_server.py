#!/usr/bin/env python3

from rclpy.duration import Duration
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

import asyncio
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from rclpy.action import ActionServer, GoalResponse, CancelResponse

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

from roverlad_interfaces.srv import GetMap
from roverlad_interfaces.srv import GetMaps
from roverlad_interfaces.action import SaveMap as SaveMapAction

from std_msgs.msg import String
from std_srvs.srv import Empty
import std_msgs.msg  as std_msgs
from nav2_msgs.srv import SaveMap

from pathlib import Path

class SlamManageServer(Node):
    def __init__(self):
        super().__init__('slam_manage_server')        


        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            lifespan=Duration(seconds=15.0)
        )

        self.getMap = self.create_service(GetMap, 'roverlad_server/get_map', self.get_map_cb)
        self.getMaps = self.create_service(GetMaps, 'roverlad_server/get_maps', self.get_maps_cb)

        # self.enableSlam = self.create_service(Empty, 'roverlad_server/enable_slam', self.enable_slam_cb)
        self.enableSlam = self.create_subscription(String, 'roverlad_server/enable_slam', self.enable_slam_cb, qos)
        self.resetMap = self.create_service(Empty, 'roverlad_server/reset_map', self.reset_slam_cb)

        self.saveMap = ActionServer(self, SaveMapAction, "roverlad_server/save_map", execute_callback=self.execute_save_map_cb,
                                                                                      goal_callback=self.goal_save_map_cb,
                                                                                  cancel_callback=self.cancel_save_map_cb,)
        

        pkg_share = get_package_share_directory("roverlad_mapping")
        self.map_general_path = Path(pkg_share) / "maps" #

        self.load_slam = False

        self.get_logger().info('slam_manage_server started')
            
    def enable_slam_cb(self, msg):
        self.load_slam = True

        self.slam_cli = self.create_client(ChangeState, '/slam_toolbox/change_state')
        self.saver_cli = self.create_client(ChangeState, '/map_saver_server/change_state')

        self.reset_pos = self.create_publisher(String, "reset_roverlad", 10)

        while not self.slam_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /slam_toolbox/change_state...')

        self.init_slam()
        self.init_mapSaver()


    def transition_state(self, cli, transition_id: int):
        req = ChangeState.Request()
        req.transition.id = transition_id
        return cli.call_async(req)

    def init_slam(self):
        fut1 = self.transition_state(self.slam_cli, Transition.TRANSITION_CONFIGURE)
        fut1.add_done_callback(self.on_slam_configured)

    def on_slam_configured(self, _):
        fut2 = self.transition_state(self.slam_cli, Transition.TRANSITION_ACTIVATE)
        fut2.add_done_callback(self.on_slam_activated)
    
    def on_slam_activated(self, _):
        self.get_logger().info('slam initialized')


    def init_mapSaver(self):
        fut1 = self.transition_state(self.saver_cli, Transition.TRANSITION_CONFIGURE)
        fut1.add_done_callback(self.on_saver_configured)

    def on_saver_configured(self, _):
        fut2 = self.transition_state(self.saver_cli, Transition.TRANSITION_ACTIVATE)
        fut2.add_done_callback(self.on_saver_activated)
    
    def on_saver_activated(self, _):
        self.get_logger().info('saver initialized')


    def goal_save_map_cb(self, goal_request):
        self.get_logger().info(f'>Attempt Save map as {goal_request.save_name}')

        if not self.load_slam:
            self.get_logger().info("SLAM is not loaded/configured!")
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def cancel_save_map_cb(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    async def execute_save_map_cb(self, goal_handle):
        result = SaveMapAction.Result()
        result.success = True

        try:
            map_dir = self.map_general_path / goal_handle.request.save_name
            map_dir.mkdir(parents=True, exist_ok=True)

            self.cli = self.create_client(SaveMap, '/map_saver_server/save_map')
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for save_map service...")

            req = SaveMap.Request()
            req.map_url = str(map_dir / "map")
            req.map_topic = "map"

            future = self.cli.call_async(req)
            response = await future

            if response.result:
                self.get_logger().info("Map saved")
                goal_handle.succeed()
            else:
                self.get_logger().error("Map save failed")
                result.success = False
                goal_handle.abort()
        except Exception as e:
            self.get_logger().info(f"Error in [execute_save_map_cb] while saving map: ${e}")
            result.success = False
            goal_handle.abort()
                   
        return result
        
    def reset_slam_cb(self, req, res):

        # Exact equivalents of ros2 lifecycle 
        fut1 = self.transition_state(self.slam_cli, Transition.TRANSITION_DEACTIVATE)
        fut1.add_done_callback(self.on_slam_deactivate)

        return res
    
    def on_slam_deactivate(self, _):
        fut2 = self.transition_state(self.slam_cli, Transition.TRANSITION_CLEANUP)  # ← erases map
        fut2.add_done_callback(self.on_slam_cleanup)

    def on_slam_cleanup(self, _):
        self.reset_pos.publish(String())
        self.init_slam()
        
        self.get_logger().info('slam cleanup')

    def get_maps_cb(self, req, res):
        res.map_names = [p.name for p in self.map_general_path.iterdir() if p.is_dir()]
        return res

    def get_map_cb(self, req, res):
        res.pgm_dir, res.yaml_dir = self.get_map(req.map_name)
        return res
        
    def get_map(self, mapName):
        map_dir = self.map_general_path / mapName

        pgm  = str(map_dir / "map.pgm")
        yaml = str(map_dir / "map.yaml")

        return (pgm, yaml)

def main():
    rclpy.init()
    node = SlamManageServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
