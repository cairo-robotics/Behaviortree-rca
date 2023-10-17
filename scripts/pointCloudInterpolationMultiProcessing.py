#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Run file roslaunch realsense2_camera rs_aligned_depth.launch to get aligned images and then query them

The code is running for joint state:
---
header: 
  seq: 2742283
  stamp: 
    secs: 1696660373
    nsecs: 505905783
  frame_id: ''
name: 
  - head_pan
  - right_j0
  - right_j1
  - right_j2
  - right_j3
  - right_j4
  - right_j5
  - right_j6
  - torso_t0
position: [0.11507421875, 0.758361328125, -0.1794775390625, -1.2341728515625, 1.64531640625, 1.351759765625, 1.23762109375, 2.53654296875, 0.0]
velocity: [-0.001, -0.001, -0.001, -0.001, -0.001, -0.001, -0.001, -0.001, 0.0]
effort: [0.0, -0.9, -30.044, -12.944, -1.352, 2.74, 0.22, 0.104, 0.0]
---

'''

import rospy,message_filters
import open3d
import cv2, os, copy
import matplotlib.pyplot as plt
import numpy as np
from sympy import Point, Ellipse, Circle, RegularPolygon
from ctypes import * # convert float to uint32
import time 

import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import std_msgs.msg

from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
from multiprocessing import Pool
from multiprocessing.pool import ThreadPool
from multiprocessing import freeze_support
                
class ptCloudNode():
    def __init__(self, name, pt1 = [10, 100], pt2 = [40,200]) -> None:
        """Initialises the depth reader node

        Args:
            name (string): base name of the topic 
        """
        self.baseName   = name
        self.topicName  = self.baseName + "image_raw"
        self.i          = 0
        self.u_min, self.u_max = pt1[0], pt2[0]
        self.v_min, self.v_max = pt1[1], pt2[1]
        # Create an ellipse of the same major and minor axis as that of the iron KET, then we try to do ICP 
        r = RegularPolygon(Point(0, 0), 0.0065, 3)
        self.e1  =   r.incircle
        ket_ellipse = [self.e1.random_point(seed=np.random.uniform(-1,1)).n().coordinates for _ in range(50)]
        self.ket_ellipse = copy.deepcopy(ket_ellipse)
        for i in ket_ellipse:
            self.ket_ellipse.append((i[0], -i[1]))

    
    def ptCloudcallback_volumeFilling(self, pointcloud):
        """
        Callback that recieved depth image message and stores an open3D point cloud from the same

        Args:
            pointcloud (sensor_msgs.image): depth image from D435 camera
        """
        
        # Depth scale: The Depth scale is 1mm. ROS convention for uint16 depth image https://github.com/IntelRealSense/realsense-ros/issues/714#issuecomment-479907272
        
        self.intrinsic_mtrx = np.array(self.cam_info.K)
        self.intrinsic_mtrx = self.intrinsic_mtrx.reshape((3,3))
        self.intrinsics_o3d = open3d.camera.PinholeCameraIntrinsic(int(self.cam_info.width), int(self.cam_info.height), self.intrinsic_mtrx)
        
        
        bridge = CvBridge()
        
        cv_image = bridge.imgmsg_to_cv2(pointcloud)
        self.open3dptCloud = open3d.geometry.PointCloud.create_from_depth_image(open3d.geometry.Image(cv_image.astype(np.uint16)), self.intrinsics_o3d) #[self.u_min:self.u_max, self.v_min:self.v_max]
        self.open3dptCloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]) # Check here http://www.open3d.org/docs/latest/tutorial/Basic/rgbd_image.html

        # # estimate radius for rolling ball
        self.open3dptCloud.estimate_normals()
        distances = self.open3dptCloud.compute_nearest_neighbor_distance()
        avg_dist = np.mean(distances)
        radius = 1.5 * avg_dist   

        self.downpcd = self.open3dptCloud.voxel_down_sample(voxel_size=0.005)
        plane_model, inliers = self.open3dptCloud.segment_plane(distance_threshold=0.0085, ransac_n = 3, num_iterations=1000) # 0.0085 determined from the dataset
        self.inlier_cloud = self.open3dptCloud.select_by_index(inliers)
        
        self.inlier_cloud.paint_uniform_color([1.0,0,0])
        self.outlier_cloud = self.open3dptCloud.select_by_index(inliers, invert=True).voxel_down_sample(voxel_size=0.005)
        
        filledPtCloud = open3d.geometry.PointCloud()
        filledPtCloud.points =  outlierCloudInterp(self.outlier_cloud, plane_model, 10)
        filledPtCloud.paint_uniform_color([1.0,0.5,0.5])
        
        open3d.visualization.draw_geometries([self.open3dptCloud, filledPtCloud])

    def dist_PtPlane(plane_model, points):
        ans = []
        distance_to_plane = lambda x, y, z, A, B, C, D: abs(A * x + B * y + C * z + D) / ((A ** 2 + B ** 2 + C ** 2) ** 0.5)
        for i in points:
            ans.append(distance_to_plane(i[0], i[1], i[2], plane_model[0], plane_model[1], plane_model[2], plane_model[3]))
        return ans

    def cannyCallback(self, rgbImg, alighnedDepthImg):
        """
        does canny edge detection to identify location of the KET block and publishes the final pose of the block

        Args:
            image (ros message image): Image message recieved from the topic
        """
        self.i                  += 11
        self.intrinsic_mtrx     = np.array(self.cam_info.K)
        self.intrinsic_mtrx     = self.intrinsic_mtrx.reshape((3,3))
        bridge                  = CvBridge()
        cv_image                = bridge.imgmsg_to_cv2(rgbImg)
        depthImg                = bridge.imgmsg_to_cv2(alighnedDepthImg)
        
        tt = time.time()
        SmoothedImage           = cv2.bilateralFilter(cv_image.astype(np.float32), sigmaColor = 200, sigmaSpace = 4, d = -1)
        SmoothedImage           = cv2.cvtColor(SmoothedImage, cv2.COLOR_BGR2HSV)
        print("Time to update images: ", time.time() - tt)
        
        SmoothedImage           = cv2.inRange(SmoothedImage, (100,0,0), (255,255,255))
        edges                   = cv2.Canny(SmoothedImage.astype(np.uint8), 50, 55)
        
        # Here calculate all the non zero points and create an image with values from original image values (r,g,b) and store the rest as 0
        edges  = edges.astype(np.float32)
        self.depthContour = (depthImg & edges.astype(np.int16)).astype(np.float32)     
        self.countourPublisher.publish(bridge.cv2_to_imgmsg(edges.astype(np.uint8)))
        tt = time.time()
        self.intrinsics_o3d = open3d.camera.PinholeCameraIntrinsic(int(self.cam_info.width), int(self.cam_info.height), self.intrinsic_mtrx)
        self.open3dptCloud = open3d.geometry.PointCloud.create_from_depth_image(open3d.geometry.Image(self.depthContour.astype(np.uint16)), self.intrinsics_o3d)
        self.open3dptCloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]) # Check here http://www.open3d.org/docs/latest/tutorial/Basic/rgbd_image.html
        ptCldArr = np.asarray(self.open3dptCloud.points)
        # max x value array:
        max_x_arr = np.argwhere(ptCldArr[:,0] == max(ptCldArr[:,0])).T[0]
        # Cropping point cloud
        alighnedbbox  = open3d.geometry.AxisAlignedBoundingBox(np.array([0, -np.inf, min(ptCldArr[:,2])]), np.array([np.inf, 0, max(ptCldArr[:,2])]))
        self.open3dptCloud.crop(alighnedbbox)
        print("Time to update Pointclouds: ", time.time() - tt)
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'camera_depth_optical_frame'
        contourPtCloud = PointCloud()
        contourPtCloud.header = header
        mean_z = np.mean(np.asarray(self.open3dptCloud.points)[:,2])
        
        for i in self.open3dptCloud.points:
            contourPtCloud.points.append(Point32(i[0], i[1], mean_z))
                
        referencePtCloud        = open3d.geometry.PointCloud()
        referencePtCloud.points = open3d.utility.Vector3dVector(np.array([np.array([i[0], i[1], mean_z]) for i in self.ket_ellipse]))
        referencePtCloud.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=50))
        
        loss            = open3d.pipelines.registration.TukeyLoss(k=0.0004)
        p2p             = open3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
        trans_init      = np.eye(4)
        trans_init[0,3] = 0.05
        trans_init[1,3] = -0.014
        trans_init[2,3] = 0.0
        trans_init[3,3] = 1.0
        threshold       = 0.5
        
        self.open3dptCloud.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=50))
        reg_p2p = open3d.pipelines.registration.registration_icp(referencePtCloud, self.open3dptCloud, threshold, trans_init,p2p)
        trans_init[:,3] =  reg_p2p.transformation[:,3]

        referencePtCloud.transform(trans_init)
        print(trans_init)
        for i in referencePtCloud.points:
            contourPtCloud.points.append(Point32(i[0], i[1], mean_z))
            
        self.pointcloudPublisher.publish(contourPtCloud)        
        self.rate.sleep()

    def readPointCloud(self):
        """Initialises node and calls subscriber
        """
        rospy.init_node("ReadAndSavePtCloud")
        self.rate = rospy.Rate(100)
        self.cam_info                   = rospy.wait_for_message(self.baseName + "camera_info", CameraInfo, timeout=10)
        image_sub                       = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_sub                       = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        self.ts                         = message_filters.TimeSynchronizer([image_sub, depth_sub], 1)

        self.pointcloudPublisher        = rospy.Publisher("/ContourPtCloud", PointCloud, queue_size=1)
        self.originalPtCloudPublisher   = rospy.Publisher("/OriginalPtCloud", PointCloud, queue_size=1)
        self.countourPublisher          = rospy.Publisher('/ContourImage', Image, queue_size=1)
        self.ts.registerCallback(self.cannyCallback)
        # rospy.Subscriber(self.topicName, Image ,self.ptCloudcallback, queue_size=1)
        # rospy.Subscriber(self.topicName, Image, self.cannyCallback, queue_size=1)


def workerInterp(x,y,z,plane_model,interp,ctr):
        direction_vector = np.array([plane_model[0], plane_model[1], plane_model[2]])
        # Normalize the direction vector
        direction_vector /= np.linalg.norm(direction_vector)
        distance_to_plane = lambda x, y, z, A, B, C, D: abs(A * x + B * y + C * z + D) / ((A ** 2 + B ** 2 + C ** 2) ** 0.5)
        interpolated_point = np.array([np.array([x,y,z]) - (ctr * distance_to_plane(x,y,z,plane_model[0], plane_model[1], plane_model[2], plane_model[3]) / (interp - 1)) * direction_vector])
        return interpolated_point
    
def outlierCloudInterp(outlier_cloud, plane_model, interp):
    interpolated_points = np.array([[]])

    for j in range(len(outlier_cloud.points)):
        print("percent_complete = ", j, len(outlier_cloud.points))
        # Calculate the position of the point
        x,y,z = outlier_cloud.points[j][0], outlier_cloud.points[j][1], outlier_cloud.points[j][2]

        with ThreadPool(10) as pool:
            counter = [(x,y,z,plane_model,interp,i) for i in range(interp)]
            for result in pool.starmap(workerInterp, counter):
                if 0 in interpolated_points.shape:
                    interpolated_points = np.concatenate((interpolated_points, result), axis = 1)
                else:
                    interpolated_points = np.concatenate((interpolated_points, result), axis = 0)  
                print("length of interpolated points: ", np.array([[x,y,z]]), result)           
    return open3d.utility.Vector3dVector(interpolated_points)

if __name__ == "__main__":
    tmp = ptCloudNode("/camera/depth/")
    tmp.readPointCloud()
    rospy.spin()