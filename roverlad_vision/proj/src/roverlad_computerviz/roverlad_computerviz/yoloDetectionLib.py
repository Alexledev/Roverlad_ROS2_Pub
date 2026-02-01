
import numpy as np
import cv2
import onnxruntime as ort
import gc
from roverlad_computerviz.load_config import ConfigLoader

class YoloOnnxDetect:
    def __init__(self, conf):
        print("Start session")


        self.sess = ort.InferenceSession(str(conf.get("model_path")), providers=["CUDAExecutionProvider"])

        self.thres = conf.get("classify_threshold")

        import torch
        from torchvision.ops import nms 

        self.torch = torch
        self.nms = nms
       
        print("End startup") 

    def shutdown(self):
        print("shutting down onnx...")
        del self.sess
        self.sess = None

        try:
            self.torch.cuda.empty_cache()
        except Exception:
            pass
        finally:
            gc.collect()
            print("gc finished")

    def getDebug(self):
        return (self.sess.get_outputs()[0].shape, self.sess.get_inputs()[0].name)

    def run(self, frame):

        # Preprocess
        img_in = self.preprocess(frame)

        # Run ONNX
        pred = self.sess.run(None, {"images": img_in})

        # Postprocess
        boxes, conf, cls = self.postprocess(pred, conf_thres=self.thres)

        # Scale back
        boxes[:, [0, 2]] -= self.pad_x
        boxes[:, [1, 3]] -= self.pad_y

        boxes /= self.ratio

        return (boxes, conf, cls)
    
    def letterbox(self, img, new_shape=640, color=(114, 114, 114)):
        h, w = img.shape[:2]

        # scale ratio (640 / largest dimension)
        r = new_shape / max(h, w)
        nh, nw = int(h * r), int(w * r)

        # resize
        resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)

        # create padded image
        padded = np.full((new_shape, new_shape, 3), color, dtype=np.uint8)
        pad_x = (new_shape - nw) // 2
        pad_y = (new_shape - nh) // 2
        padded[pad_y:pad_y+nh, pad_x:pad_x+nw] = resized

        return padded, r, pad_x, pad_y


    def preprocess(self, img: np.ndarray, size=640) -> np.ndarray:
        lb_img, self.ratio, self.pad_x, self.pad_y = self.letterbox(img, size)

        img_rgb = cv2.cvtColor(lb_img, cv2.COLOR_BGR2RGB)
        img_in = img_rgb.transpose(2, 0, 1) / 255.0
        img_in = img_in.astype(np.float32)
        img_in = np.expand_dims(img_in, 0)
        return img_in


    def postprocess(self, pred, conf_thres, iou_thres=0.45):

        # pred: list containing (1, 84, 8400)
        pred = pred[0]                       # (1, 84, 8400)
        pred = self.torch.from_numpy(pred)[0]     # (84, 8400)
        pred = pred.permute(1, 0)            # (8400, 84)

        xywh = pred[:, :4]
        class_scores = pred[:, 4:]           # (8400, 80)

        best_conf, best_class = self.torch.max(class_scores, dim=1)

        mask = best_conf > conf_thres
        xywh = xywh[mask]
        best_conf = best_conf[mask]
        best_class = best_class[mask]

        xyxy = xywh.clone()
        xyxy[:, 0] = xywh[:, 0] - xywh[:, 2] / 2
        xyxy[:, 1] = xywh[:, 1] - xywh[:, 3] / 2
        xyxy[:, 2] = xywh[:, 0] + xywh[:, 2] / 2
        xyxy[:, 3] = xywh[:, 1] + xywh[:, 3] / 2

        idx = self.nms(xyxy, best_conf, iou_thres)

        return xyxy[idx], best_conf[idx], best_class[idx]
