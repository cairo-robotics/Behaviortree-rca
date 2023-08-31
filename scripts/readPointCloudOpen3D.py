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
import numpy as np
from ctypes import * # convert float to uint32
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class ptCloudNode():
    def __init__(self, name) -> None:
        self.topicName = name
    
    def ptCloudcallback(self, pointcloud):
        cam_info = rospy.wait_for_message(self.topicName, CameraInfo)
        intrinsic_mtrx = np.array(cam_info.K)
        intrinsic_mtrx = intrinsic_mtrx.reshape((3,3))
        intrinsics_o3d = open3d.camera.PinholeCameraIntrinsic(int(cam_info.width), int(cam_info.height), intrinsic_mtrx)
        
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(pointcloud)
        cv_image.astype(np.uint16)
        
        self.open3dimg = open3d.geometry.PointCloud.create_from_depth_image(cv_image, intrinsics_o3d)

    def readPointCloud(self):
        rospy.init_node("ReadAndSavePtCloud")
        rospy.Subscriber(self.topicName, Image ,self.ptCloudcallback,queue_size=1)

if __name__ == "__main__":
    aa = ptCloudNode("/camera/depth/image_rect_raw")
    aa.readPointCloud()
    rospy.spin()
