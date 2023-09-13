import cv2
import open3d as o3d
from cv_bridge import CvBridge
import torch, time
import torch.backends.cudnn as cudnn
import numpy as np
from numpy import random
import rospy, message_filters
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
        self.trace          = False
        self.device         = "cpu"
        self.half           = False
        self.conf_thres     = 0.5
        self.iou_thres      = 0.3
        self.classes        = ''
        self.agnostic_nms   = False
        self.inferenceImagepub = rospy.Publisher('/ethernetDetector', Image, queue_size=10)
        self.ethernetPort   = []

    def initialize(self):
        # Initialize
        self.device = select_device(self.device)
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = attempt_load(self.weights, map_location=self.device)  # load FP32 model
        self.stride = int(self.model.stride.max())  # model stride
        self.imgsz = check_img_size(self.imgsz, s=self.stride)  # check img_size

        if self.trace:
            self.model = TracedModel(self.model, self.device, self.imgsz)

        if self.half:
            self.model.half()  # to FP16

        # Get names and colors
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]

    def detect(self, rgbImg, alighnedDepthImg):
        # Run inference
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, self.imgsz, self.imgsz).to(self.device).type_as(next(self.model.parameters())))  # run once
        old_img_w = old_img_h = self.imgsz
        old_img_b = 1

        t0 = time.time()
        #Load image here
        bridge      = CvBridge()
        img_plot    = bridge.imgmsg_to_cv2(rgbImg)
        img0        = bridge.imgmsg_to_cv2(rgbImg) #[:, :, ::-1] 
        img         = deepcopy(img0)
        img         = letterbox(img, self.imgsz, stride=self.stride)[0]
        img         = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img         = np.ascontiguousarray(img)
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
        print(np.shape(img))
        # Process detections
        for i, det in enumerate(pred):  # detections per image  
            gn = torch.tensor(img_plot.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img_plot.shape).round()

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
                    if int(cls) == 1: ## 1 is the label for the ethernet port
                        label = f'{self.names[int(cls)]} {conf:.2f}'
                        print("confidence: ", conf, "points: ", xyxy)
                        plot_one_box(xyxy, img_plot, label=label, line_thickness=1)
                        self.ethernetPort.append(xyxy)
                self.inferenceImagepub.publish(bridge.cv2_to_imgmsg(img_plot))
        
        # For the time being let's assume the first xyxy stored in the ethernet port
        # Given these xyxy coordinates, we can conclude that it is located 59 mm to the left of the object
        # Using open 3d we convert xyxy part into 3d cloud and entire depth image into 3d cloud, we then take
        # we then shift the left point and right point of the 3d cloud(min(x), min(y)) , (max(x), max(y)) by 59mm along the x axis,
        # and we should have our goal right in the middle of this
        
        # create overall point cloud
        cam_info = rospy.wait_for_message(self.baseName + "camera_info", CameraInfo)
        self.intrinsic_mtrx = np.array(cam_info.K)
        self.intrinsic_mtrx = self.intrinsic_mtrx.reshape((3,3))
        self.intrinsics_o3d = o3d.camera.PinholeCameraIntrinsic(int(cam_info.width), int(cam_info.height), self.intrinsic_mtrx)
        depthImg = bridge.imgmsg_to_cv2(alighnedDepthImg)
        total_ptCloud = o3d.geometry.PointCloud.create_from_depth_image(o3d.geometry.Image(depthImg.astype(np.uint16)), self.intrinsics_o3d)
        x_min, y_min = min(int(xyxy[0]), int(xyxy[2])), min(int(xyxy[1]), int(xyxy[3]))
        x_max, y_max = max(int(xyxy[0]), int(xyxy[2])), max(int(xyxy[1]), int(xyxy[3]))
        part_ptCloud = o3d.geometry.PointCloud.create_from_depth_image(o3d.geometry.Image(depthImg[x_min:x_max, y_min:y_max].astype(np.uint16)), self.intrinsics_o3d)
        # Left shifted points in the point cloud
        left_3dpt   = [min(part_ptCloud.points[:,0]) - 0.059, min(part_ptCloud.points[:,1])]
        right_3dpt  = [max(part_ptCloud.points[:,0]) - 0.059, max(part_ptCloud.points[:,1])]
        
        mean_pt     = [np.mean([left_3dpt[0], right_3dpt[0]]), np.mean([left_3dpt[1], right_3dpt[1]])] 
        
        return img0

    def startNode(self):
        """Initialises node and calls subscriber
        """
        rospy.init_node("InferenceNode")
        # rospy.Subscriber("/camera/color/image_raw", Image ,self.detect,queue_size=1)
        image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 1)
        ts.registerCallback(self.detect)


if __name__ == '__main__':
    tt = DetectHole()
    tt.initialize()
    tt.startNode()
    rospy.spin()