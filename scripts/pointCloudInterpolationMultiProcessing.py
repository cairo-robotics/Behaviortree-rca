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
import cv2, os, copy, math
import matplotlib.pyplot as plt
import numpy as np
from sympy import Point, Ellipse, Circle, RegularPolygon
from ctypes import * # convert float to uint32
import time 

import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, PoseStamped, Pose
from std_msgs.msg import Header

from tf2_msgs.msg import TFMessage
from tf.transformations import quaternion_from_euler
from scipy.spatial.transform import Rotation as R
from scipy import stats
from skimage.transform import hough_ellipse
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
        self.holdDistBottom = 0.038
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
        self.i                  += 1
        self.intrinsic_mtrx     = np.array(self.cam_info.K)
        self.intrinsic_mtrx     = self.intrinsic_mtrx.reshape((3,3))
        bridge                  = CvBridge()
        cv_image                = bridge.imgmsg_to_cv2(rgbImg)
        depthImg                = bridge.imgmsg_to_cv2(alighnedDepthImg)
        
        tt = time.time()
        SmoothedImage           = cv2.bilateralFilter(cv_image.astype(np.float32), sigmaColor = 200, sigmaSpace = 4, d = -1)
        SmoothedImage           = cv2.cvtColor(SmoothedImage, cv2.COLOR_BGR2HSV)
        print("Time to update images: ", time.time() - tt)
        
        SmoothedImage           = cv2.inRange(SmoothedImage, (180,0,0), (255,255,255))
        edges                   = cv2.Canny(SmoothedImage.astype(np.uint8), 70, 100)
        # Finding contours in the smooth edges from HSV space to fit ellipse and find the location of the KET
        contours            = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        for cc in contours[0]:
            print("Contour Area: ", cv2.contourArea(cc))
            if cv2.contourArea(cc) >= 300 and cv2.contourArea(cc) <= 450: # This condition only holds when the Sawyer arm is 90 cms above the ket
                ellipse     = cv2.fitEllipse(cc)
                depthPt     = np.flip(ellipse[0])
                print(int(depthPt[0]), int(depthPt[1]))
        
        if 'depthPt' in locals():
            pass
        else:
            # Add an internal logging thing here
            rospy.logwarn("Failed to find depth point skipping on publishing") 
            return 0
             
        modifiedDepthImage  = copy.deepcopy(depthImg)
        tableDepth          = stats.mode(depthImg.flatten()).mode[0]
        modifiedDepthImage.fill(np.int16(tableDepth))
        # Here calculate all the non zero points and create an image with values from original image values (r,g,b) and store the rest as 0
        self.depthContour                                   = (modifiedDepthImage & edges.astype(np.int16)).astype(np.float32)     
        self.depthContour[self.depthContour!=0]             = np.float32(tableDepth)
        self.depthContour[int(depthPt[0]), int(depthPt[1])] = np.float32(tableDepth)
        tt = time.time()
        self.intrinsics_o3d = open3d.camera.PinholeCameraIntrinsic(height=int(self.cam_info.height), width=int(self.cam_info.width), intrinsic_matrix=self.intrinsic_mtrx)
        self.open3dptCloud  = open3d.geometry.PointCloud.create_from_depth_image(open3d.geometry.Image(self.depthContour.astype(np.uint16)), self.intrinsics_o3d)
        
        # 3D point og ket center is depthPt in image plane we take this to 3D plane in Camera frame: 
        tempCloud                           = np.zeros_like(self.depthContour.astype(np.float32))
        tempCloud[int(depthPt[0]), int(depthPt[1])]   = np.float32(tableDepth)
        self.depthPt        = open3d.geometry.PointCloud.create_from_depth_image(open3d.geometry.Image(tempCloud.astype(np.uint16)), self.intrinsics_o3d)
        self.open3dptCloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]) # Check here http://www.open3d.org/docs/latest/tutorial/Basic/rgbd_image.html
        self.depthPt.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'camera_depth_optical_frame'
        
        contourPtCloud = PointCloud()
        contourPtCloud.header = header
                
        for i in self.open3dptCloud.points:
            contourPtCloud.points.append(Point32(i[0], i[1], i[2]))
        
        
        for i in self.depthPt.points:
            print("Found Pt: ", i[0], i[1], i[2])
            contourPtCloud.points.append(Point32(i[0], i[1], i[2]))            
        
        pose_val = PoseStamped()
        pose_val.header          = header
        pose_val.pose.position.x = self.depthPt.points[0][0]
        pose_val.pose.position.y = self.depthPt.points[0][1]
        pose_val.pose.position.z = self.depthPt.points[0][2] + self.holdDistBottom
        print("Point :   ", pose_val.pose.position.x, pose_val.pose.position.y, pose_val.pose.position.z )
        # TODO Fix here
        q = quaternion_from_euler(0, 0, 0)    
            
        pose_val.pose.orientation.x = q[0]
        pose_val.pose.orientation.y = q[1]
        pose_val.pose.orientation.z = q[2]
        pose_val.pose.orientation.w = q[3]
        
        self.countourPublisher.publish(bridge.cv2_to_imgmsg(self.depthContour.astype(np.uint8)))
        self.pickPosePublisher.publish(pose_val)
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
        self.pickPosePublisher          = rospy.Publisher("/visionFeedback/MeanValue", PoseStamped, queue_size=1)
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