import onnxruntime as ort
import numpy as np
import cv2

img = cv2.imread("/home/app/image.png")
img_resized = cv2.resize(img, (640, 640))
img_in = img_resized[:, :, ::-1].transpose(2,0,1)  # BGR→RGB, HWC→CHW
img_in = img_in / 255.0
img_in = np.expand_dims(img_in.astype(np.float32), 0)

sess = ort.InferenceSession("yolov5n.onnx", providers=["CUDAExecutionProvider"])
print("Prov: ", sess.get_providers())
output = sess.run(None, {"images": img_in})
print(len(output), [o.shape for o in output])
