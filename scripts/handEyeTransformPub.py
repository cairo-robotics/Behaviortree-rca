import rospy
import tf, json
import numpy as np
import tf2_msgs

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
while not rospy.is_shutdown():
    try:        
        (trans, rot)                    = readJsonTransfrom("/home/dt/HRIPapers/hand_eye_ws/hand_eye_calibration/good_results/calibration.json")
        br = tf.TransformBroadcaster()
        br.sendTransform((trans[0], trans[1], trans[2]),
                         rot,
                         rospy.Time.now(),
                         "right_gripper_r_finger_tip",
                         "camera_color_optical_frame")
        print(trans, rot)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
    
rospy.spin()
