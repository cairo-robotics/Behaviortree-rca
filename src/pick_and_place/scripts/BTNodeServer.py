import copy
import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from pick_and_place.srv import _gripper, _approach, _servotoPose, _retract

from tf.transformations import quaternion_slerp, quaternion_matrix
import tf
import numpy as np
import intera_interface

class CommandServer():
    def __init__(self, limb="right", hover_distance = -0.35, tip_name="right_gripper_tip"):
        rospy.init_node("CommandServer")
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
        """
        _guarded_move_to_joint_position: Moves the arm to the given joint angles

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

    def gripper_srv(self, cmd):
        """
        gripper_srv: Gripper command callback for the gripper_cmd service

        Args:
            cmd (string): Command to open or close the gripper

        Returns:
            gripper_reply (boolean): Success/Failure reply
        """
        if cmd.gripper_cmd == 'Open':
            try:
                rospy.sleep(1.0)
                self._gripper.open()
                rospy.sleep(1.0)
                return _gripper.gripperResponse(True)
            except Exception as e:
                self._loginfo("GripperNode", e)
                return _gripper.gripperResponse(False) 
        elif cmd.gripper_cmd == 'Close':
            try:
                rospy.sleep(1.0)
                self._gripper.close()
                rospy.sleep(1.0)
                return _gripper.gripperResponse(True)
            except Exception as e:
                self._loginfo("GripperNode", e)
                return _gripper.gripperResponse(False)
        else: 
            self._loginfo("GripperNode", "Invalid Command From Client: " + cmd.gripper_cmd)
            return _gripper.gripperResponse(False)
        

    def _approach(self, pose):
        """
        _approach: Slows down the arm and moves to the desired joint angle positions

        Args:
            pose (rosmsg pose): Desired pose where arm has to reach

        Returns:
            approach_reply(boolean): Success/Failure reply
        """
        approach = copy.deepcopy(pose.approach_pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        print("approach pose: ", approach)
        try:
            joint_angles = self._limb.ik_request(approach, self._tip_name)
        except Exception as e:
            self._loginfo("ApproachNode", e)
            return _approach.approachResponse(False)
        
        print("Approach joint_angles: ", joint_angles)
        try:
            self._limb.set_joint_position_speed(0.1)
        except Exception as e:
            self._loginfo("ApproachNode", e)
            return _approach.approachResponse(False)
        try:
            respValue = self._guarded_move_to_joint_position(joint_angles)
            if respValue == 'error':
                self._loginfo("ApproachNode failed as invalid pose to servo", approach.__str__())
                return _approach.approachResponse(False)

        except Exception as e:
            self._loginfo("ApproachNode", e)
            return _approach.approachResponse(False)
        try:
            self._limb.set_joint_position_speed(0.1)
        except Exception as e:
            self._loginfo("ApproachNode", e)
            return _approach.approachResponse(False)

        return _approach.approachResponse(True)

    def _servo_to_pose(self, servoToPose_msg, timeout=7.0):
        """
        _servo_to_pose: A Cartesian move

        Args:
            servoToPose_msg (rosmsg Pose): Pose to servo to in gripper's coordinate frame
            timeout (seconds):             Timeout for calling the service 
        Returns:
            servotopose_reply: Success/Failure reply
        """
        
        transform_base, transform_fk        = tf.transformations.identity_matrix(), tf.transformations.identity_matrix()
        transform_fk_msg                    = (self._limb.fk_request(self._limb.joint_angles())).pose_stamp[0].pose
        transform_fk[:3, 3]                 = np.array([transform_fk_msg.position.x, transform_fk_msg.position.y,transform_fk_msg.position.z])
        transform_fk[:3,:3]                 = quaternion_matrix(np.array([transform_fk_msg.orientation.x, transform_fk_msg.orientation.y, transform_fk_msg.orientation.z, transform_fk_msg.orientation.w]))[:3, :3]
        transform_base[:3, 3]               = np.array([servoToPose_msg.servo_to_pose.position.x,servoToPose_msg.servo_to_pose.position.y,servoToPose_msg.servo_to_pose.position.z])
        transform_base[:3,:3]               = quaternion_matrix(np.array([servoToPose_msg.servo_to_pose.orientation.x, servoToPose_msg.servo_to_pose.orientation.y, servoToPose_msg.servo_to_pose.orientation.z, servoToPose_msg.servo_to_pose.orientation.w]))[:3, :3]
        transform_base                      = np.matmul(transform_fk, np.linalg.inv(transform_base))
        print(transform_base, transform_fk_msg)
        qq                                  = tf.transformations.quaternion_from_matrix(transform_base)
        servo_to_pose                       = Pose(position=Point(
                                                        x=transform_base[0,3],
                                                        y=transform_base[1,3],
                                                        z=transform_base[2,3],
                                                    ),
                                                    orientation=Quaternion(
                                                        x=qq[0],
                                                        y=qq[1],
                                                        z=qq[2],
                                                        w=qq[3],
                                                    ))
        
        try:
            joint_angles = self._limb.ik_request(servo_to_pose, self._tip_name)
        except Exception as e:
            self._loginfo("ServoToPoseNode", e)
            return _servotoPose.servotoPoseResponse(False) 

        if joint_angles:
            try:
                self._limb.move_to_joint_positions(joint_angles, timeout=timeout)
            except Exception as e:
                self._loginfo("ServoToPoseNode", e)
                return _servotoPose.servotoPoseResponse(False) 
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
            return _servotoPose.servotoPoseResponse(False) 
        
        rospy.sleep(1.0)
        return _servotoPose.servotoPoseResponse(True) 

    def _retract(self, cmd):
        """
        _retract: Move to pose at a hoverdistance

        Args:
            cmd (Bool): Choose one of the two positions to be retracted to
                        True: Position over Ket,
                        False: Position over hole

        Returns:
            retract_reply(Bool): Success/Failure reply
        """
        # retrieve current pose from endpoint
        if cmd.retract_cmd:
            joint_angles = {'right_j0': 0.60579296875, 'right_j1': -0.9182119140625, 'right_j2': -0.5383134765625, 'right_j3': 1.7842587890625, 'right_j4': 0.396298828125, 'right_j5': 0.878787109375, 'right_j6': 3.1722001953125}
        else:
            self._limb.set_joint_position_speed(0.01)
            # joint_angles = {'right_j0': 0.0510966796875, 'right_j1': -0.9274990234375, 'right_j2': -0.9209423828125, 'right_j3': 1.7007294921875, 'right_j4': 0.4689482421875, 'right_j5': 1.095123046875, 'right_j6': 2.2665107421875}
            joint_angles = {'right_j0': 0.1605078125, 'right_j1': 0.5452626953125, 'right_j2': -1.2906455078125, 'right_j3': 1.2680205078125, 'right_j4': -1.04688671875, 'right_j5': -1.1503220703125, 'right_j6': 3.5907509765625}
            # joint_angles = {'right_j0': -0.9349599609375, 'right_j1': -0.5297138671875, 'right_j2': 0.52435546875, 'right_j3': 1.8495341796875, 'right_j4': -1.20587109375, 'right_j5': 0.4416484375, 'right_j6': 2.1454580078125}
        try:
            # To retract first move the joint to neutral position
            self._limb.move_to_neutral()
            resp = self._guarded_move_to_joint_position(joint_angles)
            return _retract.retractResponse(True)
        except Exception as e:
            self._loginfo("RetractNode", e)
            return _retract.retractResponse(False) 

    def gripper(self):
        service     =   rospy.Service("GripperCmd", _gripper.gripper, self.gripper_srv)
        return service

    def approach(self):
        service     =   rospy.Service("ApproachCmd", _approach.approach, self._approach)
        return service

    def ServoToPose(self):
        service     =   rospy.Service("ServoToPoseCmd", _servotoPose.servotoPose, self._servo_to_pose)
        return service

    def retract(self):
        service     =   rospy.Service("RetractCmd", _retract.retract, self._retract)
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