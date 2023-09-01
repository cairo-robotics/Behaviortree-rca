#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This script contains 2 functions for converting cloud format between Open3D and ROS:   
* convertCloudFromOpen3dToRos  
* convertCloudFromRosToOpen3d
where the ROS format refers to "sensor_msgs/PointCloud2.msg" type.

This script also contains a test case, which does such a thing:  
(1) Read a open3d_cloud from .pcd file by Open3D.
(2) Convert it to ros_cloud.
(3) Publish ros_cloud to topic.
(4) Subscribe the ros_cloud from the same topic.
(5) Convert ros_cloud back to open3d_cloud.
(6) Display it.  
You can test this script's function by rosrun this script.

'''

import rospy

import open3d
import cv2
import numpy as np
from ctypes import * # convert float to uint32
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class rgbThresholdingNode():
    def __init__(self, name) -> None:
        self.baseName   = name
        self.topicName  = self.baseName + "image_raw"
    
    def cannyCallback(self, image):
        cam_info = rospy.wait_for_message(self.baseName + "camera_info", CameraInfo)
        intrinsic_mtrx = np.array(cam_info.K)
        intrinsic_mtrx = intrinsic_mtrx.reshape((3,3))
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(image)
        # grayscale_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        hsl_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HLS_FULL)
        # Apply a threshold to the image
        thresholded_image = cv2.threshold(hsl_image, 128, 220, cv2.THRESH_BINARY)[1]
        edges = cv2.Canny(thresholded_image.astype(c_uint8)[:,:,1], 200, 250)
        cv2.imshow('Edges', edges)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
    def readColorImage(self):
        rospy.init_node("ReadAndSaveImge")
        rospy.Subscriber(self.topicName, Image ,self.cannyCallback,queue_size=1)

class ptCloudNode():
    def __init__(self, name) -> None:
        self.baseName   = name
        self.topicName  = self.baseName + "image_rect_raw"
    
    def ptCloudcallback(self, pointcloud):
        cam_info = rospy.wait_for_message(self.baseName + "camera_info", CameraInfo)
        intrinsic_mtrx = np.array(cam_info.K)
        intrinsic_mtrx = intrinsic_mtrx.reshape((3,3))
        intrinsics_o3d = open3d.camera.PinholeCameraIntrinsic(int(cam_info.width), int(cam_info.height), intrinsic_mtrx)
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(pointcloud)
        self.open3dptCloud = open3d.geometry.PointCloud.create_from_depth_image(open3d.geometry.Image(cv_image.astype(np.uint16)), intrinsics_o3d)
        
        self.open3dptCloud.estimate_normals()

        # estimate radius for rolling ball
        distances = self.open3dptCloud.compute_nearest_neighbor_distance()
        avg_dist = np.mean(distances)
        radius = 1.5 * avg_dist   
        #Open 3D generates a mesh to see and visualise the holes.
        #self.mesh = open3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(self.open3dptCloud, open3d.utility.DoubleVector([radius, radius * 2]))
        #open3d.visualization.draw_geometries([mesh])

    def readPointCloud(self):
        rospy.init_node("ReadAndSavePtCloud")
        rospy.Subscriber(self.topicName, Image ,self.ptCloudcallback,queue_size=1)

if __name__ == "__main__":
    aa = rgbThresholdingNode("/camera/color/")
    aa.readColorImage()
    rospy.spin()