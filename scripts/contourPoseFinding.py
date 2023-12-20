#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy,message_filters
import open3d
import cv2, os, copy, math
import matplotlib.pyplot as plt
import numpy as np
import time 
import tf
import tf2_ros
import json
import dataclasses

from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, PoseStamped, Pose
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from tf.transformations import quaternion_from_matrix
from scipy.spatial.transform import Rotation as R
from scipy import stats
from skimage.transform import hough_ellipse
from cv_bridge import CvBridge
from sympy import Point, Ellipse, Circle, RegularPolygon
from ctypes import * # convert float to uint32
from multiprocessing import Pool
from multiprocessing.pool import ThreadPool
from multiprocessing import freeze_support

@dataclasses.dataclass
class VisionProcessingData:
    """_summary_

    Returns:
        _type_: _description_
    """
    intrinsic_matrix : np.ndarray
    depth_contour    : open3d.camera.PinholeCameraIntrinsic

class PoseEstimationNode():
    """_summary_
    """
    def __init__(self, name) -> None:
        """Initialises the depth reader node

        Args:
            name (string): base name of the topic 
        """
        self.baseName   = name
        self.topicName  = self.baseName + "image_raw"
        self.i          = 0
        self.holdDistBottom = 0.035
        # Create an ellipse of the same major and minor axis
        # as that of the iron KET
        r = RegularPolygon(Point(0, 0), 0.0065, 3)
        self.e1  =   r.incircle
        ket_ellipse = [self.e1.random_point(seed=np.random.uniform(-1,1)).n().coordinates
                       for _ in range(50)]
        self.ket_ellipse = copy.deepcopy(ket_ellipse)
        for i in ket_ellipse:
            self.ket_ellipse.append((i[0], -i[1]))

        self.objects = ["ket", "bigCylinder", "smallCylinder"]

    def readJsonTransfrom(self, fileName):
        with open(fileName, 'r') as f:
            data = json.load(f)
        return (np.array([data['translation']['x'],
                          data['translation']['y'],data['translation']['z']]),
                np.array([data['rotation']['i'],data['rotation']['j'],
                          data['rotation']['k'],data['rotation']['w']]))

    def readJsonPickPosn(self, fileName):
        with open(fileName, 'r') as f:
            data = json.load(f)
        return data["BigCylinder"], data["SmallCylinder"]

    def createPose(self, homogeneous_matrix : np.ndarray, frame_id : str) -> PoseStamped:
        """_summary_

        Args:
            HomogeneousMatrix (np.ndarray): Homogeneous matrix of the transform

        Returns:
            PoseStamped: Pose message that has to be published/broadcaster, with timestamp
        """
        pose_msg = PoseStamped()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id

        pose_msg.header          = header
        pose_msg.pose.position.x = homogeneous_matrix[0][3]
        pose_msg.pose.position.y = homogeneous_matrix[1][3]
        pose_msg.pose.position.z = homogeneous_matrix[2][3]

        q = quaternion_from_matrix(homogeneous_matrix)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        return pose_msg

    def canny_callback(self, rgbImg, alighnedDepthImg):
        """does canny edge detection to identify location
        of the KET block and publishes the final pose of the block.

        Args:
            image (ros message image): Image message recieved from the topic
        """
        self.i                  += 1
        self.intrinsic_mtrx     = np.array(self.cam_info.K)
        self.intrinsic_mtrx     = self.intrinsic_mtrx.reshape((3,3))
        bridge                  = CvBridge()
        cv_image                = bridge.imgmsg_to_cv2(rgbImg)
        depthImg                = bridge.imgmsg_to_cv2(alighnedDepthImg)

        SmoothedImage           = cv2.bilateralFilter(cv_image.astype(np.float32),
                                                      sigmaColor = 200,
                                                      sigmaSpace = 4,
                                                      d = -1)
        SmoothedImage           = cv2.cvtColor(SmoothedImage, cv2.COLOR_BGR2HSV)
        SmoothedImage           = cv2.inRange(SmoothedImage, (100,0,0), (255,255,255))
        edges                   = cv2.Canny(SmoothedImage.astype(np.uint8), 70, 100)

        # Finding contours in the smooth edges from HSV space
        # to fit ellipse and find the location of the KET
        contours            = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        if len(contours[0]) == 0:
            # Add an internal logging thing here
            rospy.logwarn("Failed to find depth point skipping on publishing")
            return 0

        for cc in contours[0]:
            # This condition only holds when the Sawyer arm is 90 cms above the ket
            if cv2.contourArea(cc) >= 320 and cv2.contourArea(cc) <= 380:
                ellipse     = cv2.fitEllipse(cc)
                depthPt     = np.flip(ellipse[0])
        if 'depthPt' not in locals().keys():
            # Add an internal logging thing here
            rospy.logwarn("Object not found thus skipping.")
            return 0

        modifiedDepthImage  = copy.deepcopy(depthImg)
        tableDepth          = stats.mode(depthImg.flatten()).mode[0]
        modifiedDepthImage.fill(np.int16(tableDepth))
        # Here calculate all the non zero points
        # and create an image with values from original image values (r,g,b)
        # and store the rest as 0
        self.depthContour  = (modifiedDepthImage & edges.astype(np.int16)).astype(np.float32)

        self.depthContour[self.depthContour!=0] = np.float32(tableDepth)

        self.depthContour[int(depthPt[0]), int(depthPt[1])] = np.float32(tableDepth)

        self.intrinsics_o3d = open3d.camera.PinholeCameraIntrinsic(height=int(self.cam_info.height),
                                width=int(self.cam_info.width),
                                intrinsic_matrix=self.intrinsic_mtrx)

        self.open3dptCloud  = open3d.geometry.PointCloud.create_from_depth_image(
                                open3d.geometry.Image(
                                self.depthContour.astype(np.uint16)),
                                self.intrinsics_o3d)

        # 3D point og ket center is depthPt in image plane
        # we take this to 3D plane in Camera frame:
        tempCloud        = np.zeros_like(self.depthContour.astype(np.float32))
        tempCloud[int(depthPt[0]), int(depthPt[1])]   = np.float32(tableDepth)
        self.depthPt = open3d.geometry.PointCloud.create_from_depth_image(open3d.geometry.Image(
                                                                        tempCloud.astype(np.uint16)),
                                                                        self.intrinsics_o3d)
   
        # Check here http://www.open3d.org/docs/latest/tutorial/Basic/rgbd_image.html
        self.open3dptCloud.transform([[1, 0, 0, 0],
                                      [0, -1, 0, 0],
                                      [0, 0, -1, 0],
                                      [0, 0, 0, 1]])
        self.depthPt.transform([[1, 0, 0, 0],
                                [0, -1, 0, 0],
                                [0, 0, -1, 0],
                                [0, 0, 0, 1]])

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'camera_depth_optical_frame'

        contourPtCloud = PointCloud()
        contourPtCloud.header = header
    
        for i in self.open3dptCloud.points:
            contourPtCloud.points.append(Point32(i[0], i[1], i[2]))

        if np.asarray(self.depthPt.points).shape[0] != 0:
            for i in self.depthPt.points:
                contourPtCloud.points.append(Point32(i[0], i[1], i[2]))         
            try:
                self.PosePublisher("transforms/referencePickLocations.json",
                                   "src/pick_and_place/src/GripperToCameraTransform.json",
                                   bridge,
                                   contourPtCloud)
            except RuntimeError:
                rospy.logwarn("Failed to publish transform")

            self.countourPublisher.publish(bridge.cv2_to_imgmsg(self.depthContour.astype(np.uint8)))
            # self.pickPosePublisher.publish(pose_val)
            self.pointcloudPublisher.publish(contourPtCloud)
            self.rate.sleep()
            return
        else:
            rospy.logwarn("Failed somewhere else")
            self.countourPublisher.publish(bridge.cv2_to_imgmsg(self.depthContour.astype(np.uint8)))
            self.pointcloudPublisher.publish(contourPtCloud)
            self.rate.sleep()
            return

    def PosePublisher(self, pickLocPath : str, gripperTransformPath : str, bridge : CvBridge, contourPtCloud : PointCloud):
        BigCylinder, SmallCylinder = self.readJsonPickPosn(pickLocPath)

        # Here the angle is that of how gripper at the time of pick position
        # (the transform between camera and gripper is already math.pi/2)
        cameratoKet            = tf.transformations.euler_matrix(0,0,0,'rxyz')
        cameratoKet[:3, 3]     = np.array([self.depthPt.points[0][0],
                                           self.depthPt.points[0][1],
                                           (self.depthPt.points[0][2] + self.holdDistBottom)])
        cameratoKet            = np.linalg.inv(cameratoKet)

        # Pose for other 2 objects:
        # Here the angle is that of how gripper at the time of pick position
        # (the transform between camera and gripper is already math.pi/2)
        cameratoBigCylinder             = tf.transformations.euler_matrix(0,0,0,'rxyz')
        cameratoBigCylinder[:3, 3]      = np.array([self.depthPt.points[0][0] + BigCylinder[0],
                                                    self.depthPt.points[0][1] + BigCylinder[1],
                                                    (self.depthPt.points[0][2] + self.holdDistBottom)])
        cameratoBigCylinder             = np.linalg.inv(cameratoBigCylinder)

        # Here the angle is that of how gripper at the time of pick position
        # (the transform between camera and gripper is already math.pi/2)
        cameratoSmallCylinder           = tf.transformations.euler_matrix(0,0,0,'rxyz')
        cameratoSmallCylinder[:3, 3]    = np.array([self.depthPt.points[0][0] + SmallCylinder[0],
                                                    self.depthPt.points[0][1] + SmallCylinder[1],
                                                    (self.depthPt.points[0][2] + self.holdDistBottom)])

        cameratoSmallCylinder           = np.linalg.inv(cameratoSmallCylinder)

        pose_ket                        = self.createPose(cameratoKet,
                                                          frame_id='camera_depth_optical_frame')

        pose_BCylinder                  = self.createPose(cameratoBigCylinder,
                                                          frame_id='camera_depth_optical_frame')

        pose_SCylinder                  = self.createPose(cameratoSmallCylinder,
                                                          frame_id='camera_depth_optical_frame')
        pose_list                       = [pose_ket, pose_BCylinder, pose_SCylinder]

        obj_dict = dict(zip(self.objects, pose_list))

        try:
            (trans, rot)                    = self.readJsonTransfrom(gripperTransformPath)
            br = tf.TransformBroadcaster()
            br.sendTransform((trans[0], trans[1], trans[2]),
                            rot,
                            rospy.Time.now(),
                            "camera_depth_optical_frame",
                            "right_gripper_r_finger_tip")
            try:
                for i in obj_dict:
                    br.sendTransform((obj_dict[i].pose.position.x,
                                      obj_dict[i].pose.position.y,
                                      obj_dict[i].pose.position.z),
                                    (obj_dict[i].pose.orientation.x,
                                     obj_dict[i].pose.orientation.y,
                                     obj_dict[i].pose.orientation.z,
                                     obj_dict[i].pose.orientation.w),
                                    rospy.Time.now(),
                                    f"{i}_location",
                                    "camera_depth_optical_frame")

            except:
                self.countourPublisher.publish(
                bridge.cv2_to_imgmsg(self.depthContour.astype(np.uint8)))
                self.pointcloudPublisher.publish(contourPtCloud)
                self.rate.sleep()
                raise

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self.countourPublisher.publish(bridge.cv2_to_imgmsg(self.depthContour.astype(np.uint8)))
            # self.pickPosePublisher.publish(pose_val)
            self.pointcloudPublisher.publish(contourPtCloud) 
            self.rate.sleep()
            raise

    def ReadPointCloud(self):
        """Initialises node and calls subscriber
        """
        rospy.init_node("ReadAndSavePtCloud")
        self.rate           = rospy.Rate(30)
        self.cam_info       = rospy.wait_for_message(self.baseName + "camera_info",
                                                    CameraInfo,
                                                    timeout=10)
        image_sub           = message_filters.Subscriber('/camera/color/image_raw',
                                                        Image)
        depth_sub           = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw',
                                                        Image)
        self.ts             = message_filters.TimeSynchronizer([image_sub, depth_sub], 1)

        self.pointcloudPublisher        = rospy.Publisher("/ContourPtCloud", PointCloud, queue_size=1)
        self.originalPtCloudPublisher   = rospy.Publisher("/OriginalPtCloud", PointCloud, queue_size=1)
        self.countourPublisher          = rospy.Publisher('/ContourImage', Image, queue_size=1)
        self.pickPosePublisher          = rospy.Publisher("/visionFeedback/MeanValue", PoseStamped, queue_size=1)
        self.ts.registerCallback(self.canny_callback)

if __name__ == "__main__":
    tmp = PoseEstimationNode("/camera/depth/")
    tmp.ReadPointCloud()
    rospy.spin()