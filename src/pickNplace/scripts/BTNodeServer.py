import copy
import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from gripper.srv import gripper_cmd, gripper_reply
from approach.srv import approach_pose, approach_reply
from servotoPose.srv import servo_to_Pose, servotopose_reply
from retract.srv import retract_cmd, retract_reply

from tf.transformations import quaternion_slerp
import intera_interface 

class CommandServer():
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

    def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
        """Moves the arm to the given joint angles

        Args:
            joint_angles (_type_): _description_ #TODO: Fill the datatype
            timeout (float, optional): timeout to giving command and recieving ack from sawyer. Defaults to 5.0.

        Returns:
            string: status of servoing to the goal
        """
        if rospy.is_shutdown():
            return
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles,timeout=timeout)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
            return "error"
        return "success"

    def gripper(self, cmd):
        """Gripper command callback for the gripper_cmd service

        Args:
            cmd (string): Command to open or close the gripper

        Returns:
            gripper_reply (boolean): Success/Failure reply
        """
        if cmd == 'Open':
            try:
                self._gripper.open()
                rospy.sleep(1.0)
                return gripper_reply(True)
            except Exception as e:
                self._loginfo("GripperNode", e)
                return gripper_reply(False) 
        elif cmd == 'Close':
            try:
                self._gripper.close()
                rospy.sleep(1.0)
                return gripper_reply(True)
            except Exception as e:
                self._loginfo("GripperNode", e)
                return gripper_reply(False)
        else: 
            self._loginfo("GripperNode", "Invalid Command From Client")
            return gripper_reply(False)
        

    def _approach(self, pose):
        """Slows down the arm and moves to the desired joint angle positions

        Args:
            pose (rosmsg pose): Desired pose where arm has to reach

        Returns:
            approach_reply(boolean): Success/Failure reply
        """
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        print("approach pose: ", approach)
        try:
            joint_angles = self._limb.ik_request(approach, self._tip_name)
        except Exception as e:
            self._loginfo("ApproachNode", e)
            return approach_reply(False)
        
        print("Approach joint_angles: ", joint_angles)
        try:
            self._limb.set_joint_position_speed(0.001)
        except Exception as e:
            self._loginfo("ApproachNode", e)
            return approach_reply(False)
        try:
            self._guarded_move_to_joint_position(joint_angles)
        except Exception as e:
            self._loginfo("ApproachNode", e)
            return approach_reply(False)
        try:
            self._limb.set_joint_position_speed(0.1)
        except Exception as e:
            self._loginfo("ApproachNode", e)
            return approach_reply(False)

        return approach_reply(True)

    def _servo_to_pose(self, pose, time=4.0, steps=400.0):
        """An *incredibly simple* linearly-interpolated Cartesian move

        Args:
            pose (rosmsg Pose): Pose to servo to
            time (float, optional): Rospy rate. Defaults to 4.0.
            steps (float, optional): Steps of interpolation for the trajectory. Defaults to 400.0.

        Returns:
            servotopose_reply: Success/Failure reply
        """
        r = rospy.Rate(1/(time/steps)) # Defaults to 100Hz command rate
        try:
            current_pose = self._limb.endpoint_pose()
        except Exception as e:
            self._loginfo("ServoToPoseNode", e)
            return servotopose_reply(False) 

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
            try:
                joint_angles = self._limb.ik_request(ik_step, self._tip_name)
            except Exception as e:
                self._loginfo("ServoToPoseNode", e)
                return servotopose_reply(False) 

            if joint_angles:
                try:
                    self._limb.set_joint_positions(joint_angles)
                except Exception as e:
                    self._loginfo("ServoToPoseNode", e)
                    return servotopose_reply(False) 
            else:
                rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
                return "error"
            r.sleep()
        rospy.sleep(1.0)
        return servotopose_reply(False) 

    def _retract(self, cmd):
        """Move to pose at a hoverdistance

        Args:
            cmd (Bool): Dummy variable requested from the client

        Returns:
            retract_reply(Bool): Success/Failure reply
        """
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
        try:
            self._servo_to_pose(ik_pose)
            return retract_reply(True)
        except Exception as e:
            self._loginfo("RetractNode", e)
            return retract_reply(False) 

    def gripper(self):
        rospy.init_node("GripperCmdServer")
        service     =   rospy.service("GripperCmd", gripper_cmd, self.gripper)
        return service

    def approach(self):
        rospy.init_node("ApproachCmdServer")
        service     =   rospy.service("ApproachCmd", approach_pose, self._approach)
        return service

    def ServoToPose(self):
        rospy.init_node("ServoToPoseServer")
        service     =   rospy.service("ServoToPoseCmd", servo_to_Pose, self._servo_to_pose)
        return service

    def retract(self):
        rospy.init_node("ServoToPoseServer")
        service     =   rospy.service("ServoToPoseCmd", retract_cmd, self._servo_to_pose)
        return service

    @staticmethod
    def _loginfo(NodeName, message):
        # type: (str, str) -> None
        rospy.loginfo(f'Command Server log: ({NodeName}):  {message}')



def main():
    tt = CommandServer() 
    srv1 = tt.gripper()
    srv2 = tt.approach()
    srv3 = tt.ServoToPose()
    srv4 = tt.retract()
    rospy.spin()

main()