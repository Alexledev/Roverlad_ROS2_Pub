#!/usr/bin/env python3
# Create onnx session uninterrupted (don't import torch beforehand -> might cause CUDA conflict)
import rclpy
from roverlad_computerviz.CVSession import CVSession
from roverlad_computerviz.load_config import ConfigLoader
# from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
import math
import threading
from threading import Lock
from viz_interfaces.srv import SetRoverViz
from viz_interfaces.msg import CvMsg
from geometry_msgs.msg import Twist 

import cv2


class YoloStreamNode(Node):
    def __init__(self):
        super().__init__("yolo_stream_node")
        self.frame_count = 0
        self.snap_frame = 0

        self.setInitialCam = self.create_service(SetRoverViz, 'roverlad_cv/setCam', self.setCam)
        self.image_pub = self.create_publisher(Image, "detection_image", 10)
        self.cv_pub = self.create_publisher(CvMsg, "roverlad_cv/cvData", 10)

        self.cv = None
        self.frame_timer = None
        self.lock = None

        conf = ConfigLoader("AI", "vision")     
        self.cocoObjs = conf.get("coco_classes")

        self.warningDist = conf.get("warning_distance")
        self.dangerDist = conf.get("danger_distance")

        self.get_logger().info("Yolo Stream Node Initialized")

    def setCam(self, req, res):
        if (req.enable == True):
            return self.initialize(req, res)
        else:
            return self.stopCam(req, res)

    def stopCam(self, req, res):      
        self.cv.destroy()
        self.cv = None
        self.frame_timer.cancel()
        res.success = True
        return res

    def initialize(self, req, res) -> float:
        try:
            self.cv = CVSession()
            self.cv.setFrameCallback(self.processImage)
            self.frame_timer = self.create_timer(1.0, self.timerCallback) 
            self.lock = Lock()

            self.runCV()
            self.get_logger().info("Begin cam yo")
            res.success = True

        except Exception as error:

            self.get_logger().error(f"${error}")
            res.success = False

        finally:
            return res    

    def runCV(self):        
        self.cv_thread = threading.Thread(target=self.cv.run, daemon=True)
        self.cv_thread.start()
        

    def timerCallback(self):
        self.snap_frame = self.frame_count
        self.frame_count = 0

    def measureFPS(self, frame):
        with self.lock:
            self.frame_count+=1
            
        cv2.putText(frame, f"FPS: {self.snap_frame}", (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

    def processImage(self, frame, boxes, conf, cls):
        self.measureFPS(frame)
        
        frameWidth  = frame.shape[1]
        frameHeight = frame.shape[0]

        self.detectObj(frame, boxes, conf, cls, frameWidth, frameHeight)

        # --- Publish back as Image msg ---
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        msg.height = frameHeight
        msg.width  = frameWidth
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3   # width * channels
        msg.data = frame.tobytes()

        self.image_pub.publish(msg)

    def detectObj(self, frame, boxes, conf, cls, frameWidth, frameHeight):
                
        X1Arr = []
        Y1Arr = []
        X2Arr = []
        Y2Arr = []
        for i in range(len(boxes)):

            x1, y1, x2, y2 = boxes[i].int().tolist()
            X1Arr.append(x1)
            Y1Arr.append(y1)
            X2Arr.append(x2)
            Y2Arr.append(y2)

            label = f"{self.cocoObjs[cls[i]]} {conf[i]:.2f}"

            box_clr = (0, 255, 0)    
            if (self.calcDistance(y2, frameHeight, self.dangerDist)):
                box_clr = (0, 0, 255)              
                cv2.putText(frame, "Danger!", (int(x1 + (x2 - x1)/2 - 5), int(y1 + (y2 - y1)/2)), cv2.FONT_HERSHEY_COMPLEX, 0.5, box_clr, 1)  
            elif (self.calcDistance(y2, frameHeight, self.warningDist)):
                box_clr = (0, 255, 255)              
                cv2.putText(frame, "Warning!", (int(x1 + (x2 - x1)/2 - 5), int(y1 + (y2 - y1)/2)), cv2.FONT_HERSHEY_COMPLEX, 0.5, box_clr, 1)  
       
            cv2.rectangle(frame, (x1, y1), (x2, y2), box_clr, 2)
            cv2.putText(frame, label, (x1+2, y1+12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_clr, 1)


        msg = CvMsg()
        msg.x1 = X1Arr
        msg.y1 = Y1Arr
        msg.x2 = X2Arr
        msg.y2 = Y2Arr
                    
        msg.frame_width  = frameWidth
        msg.frame_height = frameHeight

        msg.warning_dist = self.warningDist
        msg.danger_dist = self.dangerDist

        self.cv_pub.publish(msg)

    def calcDistance(self, objY, frameHeight, threshold):
        return frameHeight - objY <= threshold

    
def main():
    rclpy.init()
    node = YoloStreamNode()
    # node.run_cv()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()