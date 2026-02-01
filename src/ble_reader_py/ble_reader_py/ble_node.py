#!/usr/bin/env python3
import asyncio
import rclpy
import numpy as np
from rclpy.node import Node
# from std_msgs.msg import String
# from std_msgs.msg import Int16MultiArray
from roverlad_interfaces.msg import ControllerData
from bleak import BleakClient, BleakScanner, BleakError

TARGET_MAC = "E8:F6:0A:8C:C4:1D"
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHAR_UUID = "beb5483e-36e1-4688-efb5-ea07361b26a8"
# CHAR2_UUID = "fc5e791b-d6e7-4f5e-ba83-971fc57581a7"

RECONNECT_DELAY = 5  # seconds between reconnect attempts

class BLENode(Node):
    def __init__(self):
        super().__init__('ble_node')
        self.publisher_ = self.create_publisher(ControllerData, 'controller_data', 10)
        self.get_logger().info("BLE Node started")

    # Notification handler
    def handler(self, sender, data):

        signed_values = np.frombuffer(data, dtype=np.int8)
    
        # Publish as a string or however you want
        valueList = signed_values.tolist()

        msg = ControllerData()
        msg.rx = valueList[0]
        msg.ry = valueList[1]
        msg.lx = valueList[2]
        msg.ly = valueList[3]
        
        self.publisher_.publish(msg)        

    async def connect_and_listen(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f"Attempting to connect to {TARGET_MAC}...")
                async with BleakClient(TARGET_MAC) as client:
                    if not client.is_connected:
                        self.get_logger().error("Failed to connect, retrying...")
                        await asyncio.sleep(RECONNECT_DELAY)
                        continue

                    self.get_logger().info("Connected!")

                    await client.start_notify(CHAR_UUID, self.handler)
                    self.get_logger().info("Subscribed to notifications")

                    # await client.start_notify(CHAR2_UUID, self.handler)
                    # self.get_logger().info("Subscribed to notifications 2")

                    # Keep running until disconnected
                    while client.is_connected and rclpy.ok():
                        await asyncio.sleep(0.1)

                    self.get_logger().warn("Disconnected from BLE device, retrying...")
            except BleakError as e:
                self.get_logger().error(f"BLE Error: {e}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")

            await asyncio.sleep(RECONNECT_DELAY)

# Synchronous wrapper for console_scripts
def main():
    asyncio.run(_main())

async def _main():
    rclpy.init()
    node = BLENode()
    try:
        await node.connect_and_listen()
    finally:
        node.destroy_node()
        rclpy.shutdown()
