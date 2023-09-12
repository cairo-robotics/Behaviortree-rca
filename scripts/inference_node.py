import argparse
import time
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from numpy import random
import rospy
from sensor_msgs.msg import Image, CameraInfo
from experimental import attempt_load
from datasets import letterbox
from general import check_img_size, non_max_suppression, scale_coords, xyxy2xywh
from plots import plot_one_box
from torch_utils import select_device, time_synchronized, TracedModel
from copy import deepcopy
import sys
sys.path.insert(0, './yolov7')

class DetectHole:
    def __init__(self) -> None:
        self.weights        = "NIST_board.pt"
        self.imgsz          = 640
        self.trace          = True
        self.device         = "cpu"
        self.half           = False
        self.conf_thres     = 0.95
        self.iou_thres      = 0.8
        self.classes        = ''
        self.agnostic_nms   = ''

    def initialize(self):
        # Initialize
        self.device = select_device(self.device)
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = attempt_load(self.weights, map_location=self.device)  # load FP32 model
        self.stride = int(self.model.stride.max())  # model stride
        self.imgsz = check_img_size(self.imgsz, s=self.stride)  # check img_size

        if self.trace:
            model = TracedModel(self.model, self.device, self.imgsz)

        if self.half:
            model.half()  # to FP16

        # Get names and colors
        self.names = model.module.names if hasattr(model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]

    def detect(self, buffer):
        # Run inference
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, self.imgsz, self.imgsz).to(self.device).type_as(next(self.model.parameters())))  # run once
        old_img_w = old_img_h = self.imgsz
        old_img_b = 1

        t0 = time.time()
        #Load image here
        bridge      = CvBridge()
        img0        = bridge.imgmsg_to_cv2(buffer)[:, :, ::-1].transpose(2, 0, 1)
        img         = deepcopy(img0)
        img         = torch.from_numpy(img).to(self.device)
        img         = img.half() if self.half else img.float()  # uint8 to fp16/32

        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Warmup
        if self.device.type != 'cpu' and (old_img_b != img.shape[0] or old_img_h != img.shape[2] or old_img_w != img.shape[3]):
            old_img_b = img.shape[0]
            old_img_h = img.shape[2]
            old_img_w = img.shape[3]
            for i in range(3):
                self.model(img)[0]

        # Inference
        t1 = time_synchronized()
        print(np.shape(img))
        with torch.no_grad():   # Calculating gradients would cause a GPU memory leak
            pred = self.model(img)[0]
        t2 = time_synchronized()

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, agnostic=self.agnostic_nms)
        t3 = time_synchronized()
        s = ''
        # Process detections
        for i, det in enumerate(pred):  # detections per image  
            gn = torch.tensor(img0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    if int(c) < len(self.names):
                        s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string
                    else: 
                        pass
                        # print("Number of classes exceeded while training retrain the network")

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    if int(cls) < len(self.names):
                        label = f'{self.names[int(cls)]} {conf:.2f}'
                        print(label)
                        # plot_one_box(xyxy, img0, label=label, color=self.colors[int(cls)], line_thickness=1)

        # Print time (inference + NMS)
        # print(f'{s}Done. ({(1E3 * (t2 - t1)):.1f}ms) Inference, ({(1E3 * (t3 - t2)):.1f}ms) NMS')
        return img0

    def startNode(self):
        """Initialises node and calls subscriber
        """
        rospy.init_node("InferenceNode")
        rospy.Subscriber("/camera/color/image_raw", Image ,self.detect,queue_size=1)

if __name__ == '__main__':
    tt = DetectHole()
    tt.initialize()
    tt.startNode()
    rospy.spin()