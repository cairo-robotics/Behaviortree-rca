import rospy
from intera_core_msgs.msg import EndpointState
from geometry_msgs.msg import Twist, WrenchStamped
from std_msgs.msg import Header
rospy.init_node("temp")
rospy.Rate(100)
counter = 0

def callback(msg):
    """Callback to publish twist

    Args:
        msg (EndpointState): _description_
    """
    global counter
    new_msg = WrenchStamped()
    new_msg.header = Header()
    new_msg.header.frame_id = "right_gripper_base"
    new_msg.wrench = msg.wrench
    print("Total force: ", (msg.wrench.force.x**2 + msg.wrench.force.y**2 + msg.wrench.force.z**2)**0.5)
    t2.publish(new_msg)
    counter += 1

t2 = rospy.Publisher("EndeffectorTwist", WrenchStamped, queue_size=1)
rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, callback)
rospy.spin()
