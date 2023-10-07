import rospy
import tf, json
import numpy as np
import tf2_msgs

def readJsonTransfrom(fileName):
    with open(fileName, 'r') as f: data = json.load(f)
    return (np.array([data['translation']['x'], data['translation']['y'],data['translation']['z']]), np.array([data['rotation']['i'],data['rotation']['j'],data['rotation']['k'],data['rotation']['w']]))

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
                         "camera_link",
                         "right_gripper_base")
        print(trans, rot)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
    
rospy.spin()
