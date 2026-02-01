import asyncio
import fractions
import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, MediaStreamTrack
import aiohttp_cors
import json
import sounddevice as sd
from load_config import ConfigLoader
import av
from av import VideoFrame
import micfinder
# from cv_interfaces.srv
from viz_interfaces.srv import SetRoverViz


class CVReceiverNode(Node):
    cvNode_ready = threading.Event()
    ros_thread = None
    cvNode = None
    def __init__(self):
        super().__init__('cv_receiver_node')

        self.callback = None
        self.setVision = self.create_client(SetRoverViz, "roverlad_cv/setCam")
        self.sub = self.create_subscription(Image, "detection_image", self.msgToImg, 10)

        
    def msgToImg(self, msg):
        frame = np.frombuffer(msg.data, dtype=np.uint8)
        frame = frame.reshape((msg.height, msg.width, 3))
        self.callback(frame)
                
    async def sendCVReq(self, enable):
        
        print("----> sendCVReq")
        while not self.setVision.wait_for_service(timeout_sec=0.1):
            print("waiting for service on roverlad_cv/setCam...")
            await asyncio.sleep(0.1)

        goal = SetRoverViz.Request()
        goal.enable = enable
        goal.ai_mode = True

        print("----> sendCVReq call async")
        response = await self.setVision.call_async(goal)

        print("----> sendCVReq got response:", response)
        return response.success
    
    @staticmethod
    def reset():
        try:
            CVReceiverNode.cvNode_ready = threading.Event()
            CVReceiverNode.cvNode.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            CVReceiverNode.ros_thread.join()
            CVReceiverNode.ros_thread = None
            CVReceiverNode.cvNode = None
        except Exception as e:
            print("exception while resetting:", e)

    # @staticmethod
    # async def requestStartCV():
    #     if (CVReceiverNode.cvNode is None):
    #         raise Exception("cvNode not instanced")
        
    #     success = await CVReceiverNode.cvNode.sendStartCVReq()
    #     return success

    @staticmethod
    def createTask():

        if (CVReceiverNode.cvNode is not None):             
            return CVReceiverNode.ros_thread
        
        def lifecycle(): 
            rclpy.init()
            try:
                CVReceiverNode.cvNode = CVReceiverNode()  
                CVReceiverNode.cvNode_ready.set()        # ← signal readiness
                rclpy.spin(CVReceiverNode.cvNode)
            finally:
                if rclpy.ok():
                    rclpy.shutdown()

        CVReceiverNode.ros_thread = threading.Thread(target=lifecycle, daemon=True)
        CVReceiverNode.ros_thread.start()

        CVReceiverNode.cvNode_ready.wait() 

        #

        return CVReceiverNode.ros_thread, CVReceiverNode.cvNode

class CameraTrack(VideoStreamTrack):
    def __init__(self, cv_node: CVReceiverNode):
        super().__init__()
        self.savedFrame = None

        cv_node.callback = self.onNewFrame
        

    def onNewFrame(self, frame):
        self.savedFrame = frame
    
    async def recv(self):
        pts, time_base = await self.next_timestamp()

        if self.savedFrame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, "About to connect...", (40, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
        else:
            frame = self.savedFrame

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)                

        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame
    
class SharedMicrophone:
    def __init__(self, device):
        self.sample_rate = 48000
        self.channels = 1
        self.chunk_samples = 480          # 10 ms
        self.chunk_bytes = 480 * 2
       
        self.stream = sd.InputStream(
            device=device, #device
            samplerate=self.sample_rate,
            channels=self.channels,
            dtype="int16",
            blocksize=self.chunk_samples,
            latency="low",
        )

        self.clients = []
        self.running = True

    def register(self):
        q = asyncio.Queue(maxsize=50)
        self.clients.append(q)
        return q

    async def run(self):
        self.stream.start()
        loop = asyncio.get_running_loop()

        while self.running:
            # sounddevice is blocking → offload to thread
            data, _ = await loop.run_in_executor(None, self.stream.read, self.chunk_samples)
            chunk = data.tobytes()

            for q in self.clients:
                if not q.full():
                    q.put_nowait(chunk)

class MicrophoneTrack(MediaStreamTrack):
    kind = "audio"

    def __init__(self, shared_mic):
        super().__init__()
        self.queue = shared_mic.register()
        self.samples_sent = 0
        self.time_base = fractions.Fraction(1, 48000)

    async def recv(self):
        chunk = await self.queue.get()

        frame = av.AudioFrame(
            format="s16",
            layout="mono",
            samples=480,
        )
        frame.sample_rate = 48000
        frame.planes[0].update(chunk)

        frame.pts = self.samples_sent
        frame.time_base = self.time_base
        self.samples_sent += 480

        return frame

class RTC_APP():
    def __init__(self):

        # Peer connections
        self.pcs = set()
        self.cvNode = None
        self.camTrack = None
        self.shared_mic = None
        self.mic_index = -1
    
    def register_pc_handlers(self, peer_connection : RTCPeerConnection):

        @peer_connection.on("connectionstatechange")
        async def on_connectionstatechange():
            print(f"Connection state: {peer_connection.connectionState}")

            if peer_connection.connectionState == "failed" or peer_connection.connectionState == "closed":
                print("Something stopped or failed, cv node state is:", self.cvNode)
                if (self.cvNode is not None):
                    success = await self.cvNode.sendCVReq(False)
                    print("Disabled rvlad_vision node with status:", success)
                    CVReceiverNode.reset()
                    app["camera_task"] = None
                    self.cvNode = None
                    self.camTrack = None

                await peer_connection.close()
                self.pcs.discard(peer_connection)

    async def offer(self, request):

        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        print(" > Connected Successfully < ")

        peer_connection = RTCPeerConnection()
        self.pcs.add(peer_connection)  
        self.register_pc_handlers(peer_connection)
        try: 
            if (self.cvNode == None):
                app["camera_task"], node = CVReceiverNode.createTask()                
                self.cvNode = node

                success = await self.cvNode.sendCVReq(True)

                print("Initialize rvlad_vision node with status:", success)
            
            if (self.camTrack == None):
                self.camTrack = CameraTrack(self.cvNode)

            peer_connection.addTrack(self.camTrack)

        except Exception as e:
                print("ERROR: No Cameras found!", e)

        try:
            if (self.shared_mic == None):
                
                self.mic_index = micfinder.choose_pipewire_device()

                if (self.mic_index != -1):
                    self.shared_mic = SharedMicrophone(self.mic_index)
                    app["mic_task"] = asyncio.create_task(self.shared_mic.run())                        

            if (self.mic_index != -1):
                peer_connection.addTrack(MicrophoneTrack(self.shared_mic))

        except Exception as e:
            print("ERROR: No Microphones found!", e)

        await peer_connection.setRemoteDescription(offer)
        answer = await peer_connection.createAnswer()
        await peer_connection.setLocalDescription(answer)

        return web.Response(
            content_type="application/json",
            text=json.dumps({"sdp": peer_connection.localDescription.sdp, "type": peer_connection.localDescription.type})
        )    

    async def on_shutdown(self, app):
        """Cleanup on shutdown"""

        print(">>> on shutdown <<<")
        # if (self.shared_camera != None):
        #     self.shared_camera.stop()

        # await camApp.on_shutdown(app)

        coros = [pc.close() for pc in self.pcs]
        await asyncio.gather(*coros)
        self.pcs.clear()
  
if __name__ == "__main__":
    streaming_site = ConfigLoader(root="sites", dir="streaming_site")
    ip = streaming_site.get("ip")
    port = streaming_site.get("port")
    allow_access = streaming_site.get("allow_access")

    app = web.Application()
    camApp = RTC_APP()

    app.router.add_post("/offer", camApp.offer)

    cors = aiohttp_cors.setup(app, defaults={
        allow_access: aiohttp_cors.ResourceOptions(
            allow_credentials=True,
            expose_headers="*",
            allow_headers="*",
            allow_methods="*",
        )}
    )

    for route in list(app.router.routes()):
        cors.add(route)

    # app.on_startup.append(camApp.on_startup)
    app.on_shutdown.append(camApp.on_shutdown)
    
    print("Starting WebRTC server on http://localhost:8081")
    web.run_app(app, host=ip, port=port)
