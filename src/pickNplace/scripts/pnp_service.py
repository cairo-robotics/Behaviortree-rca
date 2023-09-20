import copy

import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
import actionlib
from pick.msg import pickPos, response
from tf.transformations import quaternion_slerp
import intera_interface

class response():
    def __init__(self, name, status) -> None:
        self.process_name = name
        self.status = status        

class PickServer():
    def __init__(self, limb="right", hover_distance = 0.15, tip_name="right_gripper_tip"):
        self._limb_name = limb # string
        self._tip_name = tip_name # string
        self._hover_distance = hover_distance # in meters
        self._limb = intera_interface.Limb(limb)
        self._gripper = intera_interface.Gripper()
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        self.PickActionServer = actionlib.SimpleActionServer("PickAction", pickPos, self.pick)

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        print("Moving to start angles: ", start_angles)
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()

    def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
        if rospy.is_shutdown():
            return
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles,timeout=timeout)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
            return "error"
        return "success"

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        print("approach pose: ", approach)
        joint_angles = self._limb.ik_request(approach, self._tip_name)
        print("Approach joint_angles: ", joint_angles)
        self._limb.set_joint_position_speed(0.001)
        self._guarded_move_to_joint_position(joint_angles)
        self._limb.set_joint_position_speed(0.1)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = copy.deepcopy(self._limb.endpoint_pose())
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        self._servo_to_pose(ik_pose)

    def _servo_to_pose(self, pose, time=4.0, steps=400.0):
        ''' An *incredibly simple* linearly-interpolated Cartesian move '''
        r = rospy.Rate(1/(time/steps)) # Defaults to 100Hz command rate
        current_pose = self._limb.endpoint_pose()
        ik_delta = Point()
        ik_delta.x = (current_pose['position'].x - pose.position.x) / steps
        ik_delta.y = (current_pose['position'].y - pose.position.y) / steps
        ik_delta.z = (current_pose['position'].z - pose.position.z) / steps
        q_current = [current_pose['orientation'].x, 
                     current_pose['orientation'].y,
                     current_pose['orientation'].z,
                     current_pose['orientation'].w]
        q_pose = [pose.orientation.x,
                  pose.orientation.y,
                  pose.orientation.z,
                  pose.orientation.w]
        for d in range(int(steps), -1, -1):
            if rospy.is_shutdown():
                return
            ik_step = Pose()
            ik_step.position.x = d*ik_delta.x + pose.position.x 
            ik_step.position.y = d*ik_delta.y + pose.position.y
            ik_step.position.z = d*ik_delta.z + pose.position.z
            # Perform a proper quaternion interpolation
            q_slerp = quaternion_slerp(q_current, q_pose, d/steps)
            ik_step.orientation.x = q_slerp[0]
            ik_step.orientation.y = q_slerp[1]
            ik_step.orientation.z = q_slerp[2]
            ik_step.orientation.w = q_slerp[3]
            joint_angles = self._limb.ik_request(ik_step, self._tip_name)
            if joint_angles:
                self._limb.set_joint_positions(joint_angles)
            else:
                rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
                return "error"
            r.sleep()
        rospy.sleep(1.0)
        return "success"

    def start(self):
        self._loginfo('PickServer Started')
        self.PickActionServer.start()
    
    def stop(self):
        self._loginfo('PickServer Stopped')

    def pick(self, pose):
        if rospy.is_shutdown():
            return
        try:
            # open the gripper
            self.gripper_open()
        except Exception as e:
            self._loginfo(f'Gripper did not open check configuration:{e}')
            result = response()
            result.success = False
            self.ReachGoalActionServer.set_aborted(result)
        
        try:
            # servo above pose
            self._approach(pose)
        except:
            self._loginfo(f'Error in approaching:{e}')
            result = response()
            result.success = False
            self.ReachGoalActionServer.set_aborted(result)
        
        try:    
            # servo to pose
            self._servo_to_pose(pose)
            if rospy.is_shutdown():
                return
        except:
            self._loginfo(f'Error in getting to object:{e}')
            result = response()
            result.success = False
            self.ReachGoalActionServer.set_aborted(result)
        
        try:    
            # close gripper
            self.gripper_close()
        except:
            self._loginfo(f'Error in closing gripper:{e}')
            result = response()
            result.success = False
            self.ReachGoalActionServer.set_aborted(result)
        
        try:    
            # retract to clear object
            self._retract()
        except:
            self._loginfo(f'Error in retracting from the object:{e}')
            result = response()
            result.success = False
            self.ReachGoalActionServer.set_aborted(result)

    def place(self, pose):
        if rospy.is_shutdown():
            return
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        if rospy.is_shutdown():
            return
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()
    
    def pick_srv_callback(self, request):
        return self.pick(request.pose)

    def pick_srv(self, pickPos):
        rospy.init_node("PickService")
        service     =   rospy.service("Picking", pickPos, self.pick_srv_callback)
        return service

    def place_srv_callback(self, request):
        return self.pick(request.pose)

    def place_srv(self, placePos):
        rospy.init_node("PickService")
        service     =   rospy.service("Placing", placePos, self.place_srv_callback)
        return service
    
    def move_to_joint_posn_callback(self, joint_angles):
        return self.move_to_start(joint_angles)
    
    def move_to_joint_posn_srv(self, joint_angles):
        rospy.init_node("MoveToJointPosn")
        service     =   rospy.service("MoveToJointPosn", joint_angles, self.move_to_joint_posn_callback)
        return service

def main():
    tt = PickAndPlace() 
    srv1 = tt.pick_srv()
    srv2 = tt.place_srv()
    srv3 = tt.move_to_joint_posn_srv()
    rospy.spin()

main()