
import threading
import cv2
from roverlad_computerviz.load_config import ConfigLoader

from yoloDetectionLib import YoloOnnxDetect

class CVSession:
    yoloProc = None

    def __init__(self):    
        conf_device = ConfigLoader("devices", "camera")      

        conf_ai = ConfigLoader("AI", "vision")       

        if (CVSession.yoloProc is None):
            CVSession.yoloProc = YoloOnnxDetect(conf_ai)            
        
        self.frameCallback = None  
        self.cap = None  
        self.device = int(conf_device.get("index"))
        self.width, self.height = map(int, conf_device.get("resolution").split("x"))
    
    def setFrameCallback(self, callback):
        self.frameCallback = callback

    def run(self):

        self.cap = cv2.VideoCapture(self.device)  # or "/dev/video0"

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)        

        
        self.actWidth  =  self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.actHeight =  self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.actFps    =  self.cap.get(cv2.CAP_PROP_FPS)



        if not self.cap.isOpened():
            raise RuntimeError("Cannot open camera")

        try:
            while self.cap is not None:


                ret, frame = self.cap.read()
                
                if not ret:
                    break

                # h, w = frame.shape[:2]
                # print(f"width={w}, height={h}")
                # print(f"actual width: {self.actWidth}, actual height: {self.actHeight}, actual FPS: {self.actFps}")
                boxes, conf, cls = CVSession.yoloProc.run(frame)

                if (self.frameCallback != None):
                    self.frameCallback(frame, boxes, conf, cls)

        except KeyboardInterrupt:
            pass
        finally:
            if (self.cap is not None):            
                self.cap.release()
       
    def destroy(self):
        self.cap.release()
        self.cap = None
        self.frameCallback = None
        print("capture down")
