import rospy
import tf, json
import numpy as np
import tf2_msgs
from geometry_msgs.msg import Point32, PoseStamped, Pose


def readJsonTransfrom(fileName):
    with open(fileName, 'r') as f: data = json.load(f)
    return (np.array([data['translation']['x'], data['translation']['y'],data['translation']['z']]), np.array([data['rotation']['i'],data['rotation']['j'],data['rotation']['k'],data['rotation']['w']]))

def inverseTransformSave(fileName):
    cameraToGripper        = np.eye(4)
    (trans, rot)           = readJsonTransfrom("/home/dt/HRIPapers/hand_eye_ws/hand_eye_calibration/good_results/calibration.json")
    cameraToGripper        = tf.transformations.quaternion_matrix(rot)
    cameraToGripper[:3, 3] = trans
    cameraToGripper        = np.linalg.inv(cameraToGripper)
    quat_To_Save           = tf.transformations.quaternion_from_matrix(cameraToGripper)
    trans_To_Save          = cameraToGripper[:3, 3]
    tot_pose               = np.concatenate([trans_To_Save, quat_To_Save])
    
    data = {"CalibrationPose": tot_pose.tolist()}
    with open(fileName, 'w') as f:
        json.dump(data, fp = f)

# inverseTransformSave("GripperToCameraTransform.json")

rospy.init_node('hand_eye_calib_transform')

listener = tf.TransformListener()

rate = rospy.Rate(1000.0)
published_pose = None
while not rospy.is_shutdown():
    try:        
        (trans, rot)                    = readJsonTransfrom("/home/dt/HRIPapers/hand_eye_ws/hand_eye_calibration/good_results/calibration.json")
        br = tf.TransformBroadcaster()
        br.sendTransform((trans[0], trans[1], trans[2]),
                         rot,
                         rospy.Time.now(),
                         "camera_depth_optical_frame",
                         "right_gripper_r_finger_tip")
        print(trans, rot)
        try: 
            published_pose = rospy.wait_for_message("/visionFeedback/MeanValue", PoseStamped, timeout=1)
            ket_publisher = tf.TransformBroadcaster()
            ket_publisher.sendTransform((published_pose.pose.position.x, published_pose.pose.position.y, published_pose.pose.position.z),
                                        (published_pose.pose.orientation.x, published_pose.pose.orientation.y, published_pose.pose.orientation.z, published_pose.pose.orientation.w),
                                        rospy.Time.now(),
                                        "ket_location",
                                        "camera_depth_optical_frame")
        except:
            continue
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    
rospy.spin()
