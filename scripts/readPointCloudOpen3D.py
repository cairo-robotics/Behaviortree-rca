#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Run file roslaunch realsense2_camera rs_aligned_depth.launch to get aligned images and then query them

'''

import rospy
import open3d
import cv2, os
import numpy as np
from ctypes import * # convert float to uint32
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge

class rgbThresholdingNode():
    def __init__(self, name) -> None:
        self.baseName   = name
        self.topicName  = self.baseName + "image_raw"
        self.i = 0
    
    def cannyCallback(self, image):
        """does canny edge detection to identify hole in which the peg is to be placed

        Args:
            image (ros message image): Image message recieved from the topic
        """
        self.i              += 1
        cam_info            = rospy.wait_for_message(self.baseName + "camera_info", CameraInfo)
        intrinsic_mtrx      = np.array(cam_info.K)
        intrinsic_mtrx      = intrinsic_mtrx.reshape((3,3))
        bridge              = CvBridge()
        cv_image            = bridge.imgmsg_to_cv2(image)
        hsl_image           = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HLS_FULL)
        thresholded_image   = cv2.threshold(hsl_image, 128, 220, cv2.THRESH_BINARY)[1]
        self.edges          = cv2.Canny(thresholded_image.astype(c_uint8)[:,:,1], 200, 250)
        cv2.imwrite('/'.join(os.getcwd().split("/")[:-2]) + "/data/images/" + "image" + str(self.i) + ".jpeg", cv_image)

    def readColorImage(self):
        """Initialises color rgb to identify hole using canny edge detection and YOLO

        Returns:
            pt1, pt2: rectangular points to the desired hole, in RGB image frame 
        """
        rospy.init_node("ReadAndSaveImge")
        rospy.Subscriber(self.topicName, Image ,self.cannyCallback,queue_size=1) # Replace this with wait for message and just get one code to run
        #TODO: Use canny image and YOLO to find appropriate depth and position in the map and return that so that point cloud node can use the same 
        # return pt1, pt2
                
class ptCloudNode():
    def __init__(self, name, pt1 = [10, 100], pt2 = [40,200]) -> None:
        """Initialises the depth reader node

        Args:
            name (string): base name of the topic 
        """
        self.baseName   = name
        self.topicName  = self.baseName + "image_raw"
        self.u_min, self.u_max = pt1[0], pt2[0]
        self.v_min, self.v_max = pt1[1], pt2[1]
    
    def ptCloudcallback(self, pointcloud):
        """
        Callback that recieved depth image message and stores an open3D point cloud from the same

        Args:
            pointcloud (sensor_msgs.image): depth image from D435 camera
        """
        
        # Depth scale: The Depth scale is 1mm. ROS convention for uint16 depth image https://github.com/IntelRealSense/realsense-ros/issues/714#issuecomment-479907272
        
        cam_info = rospy.wait_for_message(self.baseName + "camera_info", CameraInfo)
        self.intrinsic_mtrx = np.array(cam_info.K)
        self.intrinsic_mtrx = self.intrinsic_mtrx.reshape((3,3))
        self.intrinsics_o3d = open3d.camera.PinholeCameraIntrinsic(int(cam_info.width), int(cam_info.height), self.intrinsic_mtrx)
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(pointcloud)
        self.open3dptCloud = open3d.geometry.PointCloud.create_from_depth_image(open3d.geometry.Image(cv_image[self.u_min:self.u_max, self.v_min:self.v_max].astype(np.uint16)), self.intrinsics_o3d)
        self.open3dptCloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]) # Check here http://www.open3d.org/docs/latest/tutorial/Basic/rgbd_image.html
        # self.open3dptCloud.estimate_normals()

        # # estimate radius for rolling ball
        # distances = self.open3dptCloud.compute_nearest_neighbor_distance()
        # avg_dist = np.mean(distances)
        # radius = 1.5 * avg_dist   
        # self.depthAlignment()
        #Open 3D generates a mesh to see and visualise the holes.
        #self.mesh = open3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(self.open3dptCloud, open3d.utility.DoubleVector([radius, radius * 2]))
        open3d.visualization.draw_geometries([self.open3dptCloud])

    def depthAlignment(self):
        """
            Aligns rgb and depth camera images 
        """
        ### Steps to do it :
        #   Transform depth image to pt cloud 
        #   Apply rigid body transform to the point cloud to bring it to camera frame
        #   reproject 3d points to 2d image and find correspondances
        #   return the corresponding depth values
        transform_tree = rospy.wait_for_message("/tf_static", TFMessage)
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        # tf_depth_to_image = tf_buffer.lookup_transform(target_frame = "camera_aligned_depth_to_color_frame", source_frame = "camera_link", time = transform_tree.transforms[0].header.stamp)
        tf_depth_to_image = transform_tree.transforms[4] #TODO Add 
        T_depth_to_image = np.eye(4)
        T_depth_to_image[:3, :3]   = R.from_quat([tf_depth_to_image.transform.rotation.x, tf_depth_to_image.transform.rotation.y, tf_depth_to_image.transform.rotation.z, tf_depth_to_image.transform.rotation.w]).as_matrix()
        T_depth_to_image[0, 3]     = tf_depth_to_image.transform.translation.x # X translation
        T_depth_to_image[1, 3]     = tf_depth_to_image.transform.translation.y # Y translation
        T_depth_to_image[2, 3]     = tf_depth_to_image.transform.translation.z # Z translation
        # Transformed the point cloud to the depth frame
        self.open3dptCloud.transform(T_depth_to_image)
        # https://www.tangramvision.com/blog/camera-modeling-exploring-distortion-and-distortion-models-part-iii
        

    def readPointCloud(self):
        """Initialises node and calls subscriber
        """
        rospy.init_node("ReadAndSavePtCloud")
        tt = rospy.wait_for_message(self.topicName, Image)
        self.ptCloudcallback(tt)
        # rospy.Subscriber(self.topicName, Image ,self.ptCloudcallback,queue_size=1)

if __name__ == "__main__":
    tmp = ptCloudNode("/camera/aligned_depth_to_color/")
    tmp.readPointCloud()
    rospy.spin()