#!/usr/bin/env python3

import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient 
from roverlad_interfaces.action import SetSpeed
from roverlad_interfaces.action import GetSpeed
from roverlad_interfaces.action import SetSaveSpeeds
from roverlad_interfaces.srv import GetSpeedConfig
from roverlad_interfaces.srv import GetSpeedConfigs
from roverlad_interfaces.srv import SetInitialPose
from geometry_msgs.msg import PoseWithCovarianceStamped

from ament_index_python.packages import get_package_share_directory

import subprocess
import tempfile
import yaml
import asyncio
import os

class ConfigNodeServer(Node):
    def __init__(self):
        super().__init__('config_node_server')
        self.param_client = AsyncParameterClient(self, '/controller_server')        

        self.getSpeed = ActionServer(self, GetSpeed, "roverlad_server/get_speed", execute_callback=self.execute_get_speed_cb,
                                                                                  goal_callback=self.goal_get_speed_cb,
                                                                                  cancel_callback=self.cancel_get_speed_cb,)
        
        self.setSpeed = ActionServer(self, SetSpeed, "roverlad_server/set_speed", execute_callback=self.execute_set_speed_cb,
                                                                                  goal_callback=self.goal_set_speed_cb,
                                                                                  cancel_callback=self.cancel_set_speed_cb,)
        
        self.setSaveSpeeds = ActionServer(self, SetSaveSpeeds, "roverlad_server/set_save_speeds", execute_callback=self.execute_set_save_speeds_cb,
                                                                                                  goal_callback=self.goal_set_save_speeds_cb,
                                                                                                  cancel_callback=self.cancel_set_save_speeds_cb,)
        
        self.readLinearYAML = self.create_service(GetSpeedConfig, 'roverlad_server/get_linear_config', self.get_linear_config_cb)
        self.readLinearAngularYAML = self.create_service(GetSpeedConfigs, 'roverlad_server/get_linear_angular_config', self.get_linear_angular_config_cb)
        self.setInitialPose = self.create_service(SetInitialPose, 'roverlad_server/set_initial_pose', self.set_initial_pose)

        self.posePublisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.CONTROLLER_YAML = os.path.join(get_package_share_directory("roverlad_navigation"), "config", "controller_server.yaml")

        self._ready_event = asyncio.Event()      
        self._wait_task = None
        self.linear = None

        self.get_logger().info('config_node_server started')

    def goal_get_speed_cb(self, goal_request):
        self.get_logger().info(f'>Attempt Get {goal_request.name} speed')

        if goal_request.name not in ("linear", "angular"):
            self.get_logger().info("Invalid speed type")
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def cancel_get_speed_cb(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    async def execute_get_speed_cb(self, goal_handle):
        result = GetSpeed.Result()

        if (goal_handle.request.name == "linear"):
            result.spd = await self.get_linear()
        elif (goal_handle.request.name == "angular"):
            result.spd = await self.get_angular()
        else:
            self.get_logger().info("can't get unknown speed type?")     
            goal_handle.abort()

            result.spd = -1
            return result
           
        goal_handle.succeed()
        return result
    
    async def get_linear(self) -> float:
        future = self.param_client.get_parameters(['FollowPath.desired_linear_vel'])  
        await future
        return future.result().values[0].double_value

    async def get_angular(self) -> float:
        future = self.param_client.get_parameters(['FollowPath.max_angular_accel'])
        await future
        return future.result().values[0].double_value


    def goal_set_speed_cb(self, goal_request):
        self.get_logger().info(f'>Attempt Set {goal_request.name} speed to: {goal_request.value}')

        if goal_request.name not in ("linear", "angular"):
            self.get_logger().info("Invalid speed type")
            return GoalResponse.REJECT
        elif goal_request.value < 0:
            self.get_logger().info("Invalid speed value")
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def cancel_set_speed_cb(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    async def execute_set_speed_cb(self, goal_handle):
        result = SetSpeed.Result()

        if (goal_handle.request.name == "linear"):
            result.valid = await self.set_linear(goal_handle.request.value)
        elif (goal_handle.request.name == "angular"):
            result.valid = await self.set_angular(goal_handle.request.value)
        else:
            self.get_logger().info("can't set unknown speed type?")     
            goal_handle.abort()

            result.valid = False
            return result
           
        goal_handle.succeed()
        return result    

    async def set_linear(self, speed: float):
        param = Parameter(name='FollowPath.desired_linear_vel', value=speed)
        response = await self.param_client.set_parameters([param])

        result = response.results[0]

        if not result.successful:
            self.get_logger().error(f"Failed to set parameter: {result.reason}")

        return result.successful
    
    async def set_angular(self, angular: float):
        param = Parameter(name='FollowPath.max_angular_accel', value=angular)
        response = await self.param_client.set_parameters([param])

        result = response.results[0]

        if not result.successful:
            self.get_logger().error(f"Failed to set parameter: {result.reason}")

        return result.successful


    def goal_set_save_speeds_cb(self, goal_request):
        self.get_logger().info(f'>Attempt Set and Save linear {goal_request.linear} and angular {goal_request.angular}')

        if goal_request.linear < 0 or goal_request.angular < 0:
            self.get_logger().info("Invalid speed value")
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def cancel_set_save_speeds_cb(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    async def execute_set_save_speeds_cb(self, goal_handle):
        result = SetSaveSpeeds.Result()

        linearSet = await self.set_linear(goal_handle.request.linear)
        angularSet = await self.set_angular(goal_handle.request.angular)

        result.valid = linearSet and angularSet
        if (not result.valid):
            self.get_logger().info("couldn't set angular and linear speeds")    
            goal_handle.abort()

        try:        
            self.save_controller_yaml()  
        except Exception as e:
            self.get_logger().info(f"Something went wrong while saving into the controller yaml: {e}")    
            result.valid = False
            goal_handle.abort()  
  
        self.get_logger().info(f"Successfully set speeds")    
        goal_handle.succeed()    
        return result

    def save_controller_yaml(self):
        dir_path = os.path.dirname(self.CONTROLLER_YAML)

        with tempfile.NamedTemporaryFile(mode="w", dir=dir_path, delete=False) as tmp:
            subprocess.run(["ros2", "param", "dump", "/controller_server"], stdout=tmp, check=True)
            tmp_path = tmp.name

        if os.path.getsize(tmp_path) == 0:
            raise RuntimeError("Param dump produced empty file")

        os.replace(tmp_path, self.CONTROLLER_YAML)  # atomic

    def get_linear_config_cb(self, req, res) -> float:

        self.get_logger().info("Getting linear config")

        try:

            with open(self.CONTROLLER_YAML, "r") as f:
                data = yaml.safe_load(f)

            self.get_logger().info("Got yaml")

            controller = data.get("controller_server") or data.get("/controller_server")

            res.spd = controller["ros__parameters"]["FollowPath"]["desired_linear_vel"]

            self.get_logger().info(f"Spd: ${res.spd}")

        except Exception as error:

            self.get_logger().error(f"${error}")

        finally:
            return res
    
    def get_linear_angular_config_cb(self, req, res):

        with open(self.CONTROLLER_YAML, "r") as f:
            data = yaml.safe_load(f)

        controller = data.get("controller_server") or data.get("/controller_server")

        res.linear  = controller["ros__parameters"]["FollowPath"]["desired_linear_vel"]
        res.angular = controller["ros__parameters"]["FollowPath"]["max_angular_accel"]
        return res


    def create_task(self):                
        if self._wait_task is None : #or self._wait_task.done()
            self.get_logger().warning("> Create wait task")
            self._wait_task = asyncio.create_task(self.wait_for_controller())
    
    async def wait_for_controller(self):
        while not self.param_client.wait_for_services(timeout_sec=1.0):
            self.get_logger().info("Waiting for controller_server...")
            await asyncio.sleep(1.0)

        self.get_logger().info("Found controller_server")
        self._ready_event.set()
      
    async def wait_ready(self):
        self.get_logger().info("Find controller_server")

        if not self._ready_event.is_set():
            self.get_logger().info("Finding controller_server")
            self.create_task()

        await self._ready_event.wait()

    def set_initial_pose(self, req, res):       

        pos = PoseWithCovarianceStamped()
        pos.header.frame_id = 'map'
        pos.pose.pose.position.x = req.x
        pos.pose.pose.position.y = req.y
        pos.pose.pose.position.z = req.z
        pos.pose.pose.orientation.w = 1.0
        pos.pose.pose.orientation.z = req.rot
        self.posePublisher.publish(pos)

        res.success = True
        return res

def main():
    rclpy.init()
    node = ConfigNodeServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()