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
import tf, time
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
        GuardedMoveToJointPositionServer: Moves the arm to the given joint angles

        Args:
            joint_angles (np.array): Joint angles to give to the controller.
            timeout (float, optional): timeout to giving command and recieving ack from sawyer. Defaults to 5.0.

        Returns:
            string: status of servoing to the goal
        """
        self._loginfo("[GuardedMoveToJointPositionServer]", "Started Executing guarded move to joint position, timeout = 5.0")
        if rospy.is_shutdown():
            return
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles,timeout=timeout)
            self._loginfo("[GuardedMoveToJointPositionServer]", "Successfully executed guarded move to joint position")
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
            return "error"
        return "success"

    def gripper_srv(self, cmd):
        """
        GripperServiceServer: Gripper command callback for the gripper_cmd service

        Args:
            cmd (string): Command to open or close the gripper

        Returns:
            gripper_reply (boolean): Success/Failure reply
        """
        self._loginfo("[GripperServiceServer]", f"Started Executing Gripper {cmd.gripper_cmd}")
        if cmd.gripper_cmd == 'Open':
            try:
                rospy.sleep(1.0)
                self._gripper.open()
                rospy.sleep(1.0)
                self._loginfo("[GripperServiceServer]",
                              f"Successfully executed gripper {cmd.gripper_cmd} command"
                              )
                return _gripper.gripperResponse(True)
            except Exception as e:
                self._loginfo("[GripperServiceServer]", e)
                return _gripper.gripperResponse(False)
        elif cmd.gripper_cmd == 'Close':
            try:
                rospy.sleep(1.0)
                self._gripper.close()
                rospy.sleep(1.0)
                self._loginfo("[GripperServiceServer]",
                              f"Successfully executed gripper {cmd.gripper_cmd} command"
                              )
                return _gripper.gripperResponse(True)
            except Exception as e:
                self._loginfo("[GripperServiceServer]", e)
                return _gripper.gripperResponse(False)
        else: 
            self._loginfo("[GripperServiceServer]", "Invalid Command From Client: " + cmd.gripper_cmd)
            return _gripper.gripperResponse(False)


    def _approach(self, pose):
        """
        ApproachServiceServer: Slows down the arm and moves to the desired joint angle positions

        Args:
            pose (rosmsg pose): Desired pose where arm has to reach

        Returns:
            approach_reply(boolean): Success/Failure reply
        """
        approach = copy.deepcopy(pose.approach_pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        self._loginfo("[ApproachServiceServer]", "Slows down the arm and moves to the desired joint angle positions")
        try:
            self._loginfo("[ApproachServiceServer]", f"Calculating joint angles using inbuilt ik service for pose: {pose}")
            joint_angles = self._limb.ik_request(approach)
        except Exception as e:
            self._loginfo("[ApproachServiceServer]", e)
            return _approach.approachResponse(False)
        
        print("Approach joint_angles: ", joint_angles)
        try:
            self._limb.set_joint_position_speed(0.1)
            self._loginfo("[ApproachServiceServer]", "Slowed down the joint speed to: 0.1")
        except Exception as e:
            self._loginfo("[ApproachServiceServer]", e)
            return _approach.approachResponse(False)
        try:
            self._loginfo("[ApproachServiceServer]", "Calling guardedMoveToJointPosition")
            respValue = self._guarded_move_to_joint_position(joint_angles)
            if respValue == 'error':
                self._loginfo("[ApproachServiceServer] failed as invalid pose to servo", approach.__str__())
                return _approach.approachResponse(False)

        except Exception as e:
            self._loginfo("[ApproachServiceServer]", e)
            return _approach.approachResponse(False)
        try:
            self._limb.set_joint_position_speed(0.1)
            self._loginfo("[ApproachServiceServer]", "Slowed down the joint speed to: 0.1")
        except Exception as e:
            self._loginfo("[ApproachServiceServer]", e)
            return _approach.approachResponse(False)

        return _approach.approachResponse(True)

    def _servo_to_pose(self, poseNameTfTree_msg, timeout=7.0):
        """
        ServoToPoseServiceServer: A Cartesian move

        Args:
            poseNameTfTree_msg (rosmsg str): Name of transform in transform tree 
            timeout (seconds):               Timeout for calling the service 
        Returns:
            servotopose_reply: Success/Failure reply
        """
        self._loginfo("[ServoToPoseServiceServer]", "Querying desired pose in base frame of sawyer")
        listener = tf.TransformListener()
        tries = 0
        time.sleep(5)
        
        while(tries < 5):
            try: 
                (trans,rot) = listener.lookupTransform('/right_gripper_tip', poseNameTfTree_msg.servo_to_pose, rospy.Time(0))
                Tmat_right_gripper_tip_Ket = tf.transformations.quaternion_matrix(rot)
                Tmat_right_gripper_tip_Ket[:3, 3] = trans
                Tmat_fk_msg     = self._limb.fk_request(self._limb.joint_angles()).pose_stamp[0]
                (trans, rot)    = ([Tmat_fk_msg.pose.position.x, Tmat_fk_msg.pose.position.y, Tmat_fk_msg.pose.position.z], [Tmat_fk_msg.pose.orientation.x, Tmat_fk_msg.pose.orientation.y, Tmat_fk_msg.pose.orientation.z, Tmat_fk_msg.pose.orientation.w])
                Tmat_fk = tf.transformations.quaternion_matrix(rot)
                Tmat_fk[:3, 3]  = trans
                Tmat_final      = np.matmul(Tmat_fk, Tmat_right_gripper_tip_Ket)
                trans           = Tmat_final[:3, 3]
                rot             = tf.transformations.quaternion_from_matrix(Tmat_final)
                break
            except Exception as e:
                self._loginfo("[ServoToPoseServiceServer]", "Failed to fetch the transform between desired position and base.")
                time.sleep(1)
                tries += 1
                continue
            
        servo_to_pose                       = Pose(position=Point(
                                                        x=trans[0],
                                                        y=trans[1],
                                                        z=trans[2],
                                                    ),
                                                    orientation=Quaternion(
                                                        x=rot[0],
                                                        y=rot[1],
                                                        z=rot[2],
                                                        w=rot[3],
                                                    ))
        try:
            self._loginfo("[ServoToPoseServiceServer]", "Calling ik service to ")
            joint_angles = self._limb.ik_request(servo_to_pose)
        except Exception as e:
            self._loginfo("[ServoToPoseServiceServer]", e)
            return _servotoPose.servotoPoseResponse(False)

        if joint_angles:
            try:
                self._limb.move_to_joint_positions(joint_angles, timeout=timeout)
            except Exception as e:
                self._loginfo("[ServoToPoseServiceServer]", e)
                return _servotoPose.servotoPoseResponse(False)
        else:
            self._loginfo("[ServoToPoseServiceServer]",
                          "No Joint Angles provided for move_to_joint_positions. Staying put.")
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
            return _servotoPose.servotoPoseResponse(False)

        rospy.sleep(1.0)
        return _servotoPose.servotoPoseResponse(True)

    def _retract(self, cmd):
        """
        RetractServiceServer: Move to pose at a hoverdistance

        Args:
            cmd (Bool): Choose one of the two positions to be retracted to
                        True: Position over Ket,
                        False: Position over AprilTag

        Returns:
            retract_reply(Bool): Success/Failure reply
        """
        # retrieve current pose from endpoint
        if cmd.retract_cmd:
            joint_angles = {'right_j0': 0.60579296875,
                            'right_j1': -0.9182119140625,
                            'right_j2': -0.5383134765625,
                            'right_j3': 1.7842587890625,
                            'right_j4': 0.396298828125,
                            'right_j5': 0.878787109375,
                            'right_j6': 3.1722001953125
                            }
        else:
            self._limb.set_joint_position_speed(0.01)

            joint_angles = {'right_j0': 0.1835791015625,
                            'right_j1': -0.61730859375,
                            'right_j2': -1.21795703125,
                            'right_j3': 1.5466455078125,
                            'right_j4': 0.784220703125,
                            'right_j5': 1.3044609375,
                            'right_j6': 2.15759375
                            }
        try:
            # To retract first move the joint to neutral position
            self._limb.move_to_neutral()
            resp = self._guarded_move_to_joint_position(joint_angles)
            return _retract.retractResponse(True)
        except Exception as e:
            self._loginfo("RetractNode", e)
            return _retract.retractResponse(False)

    def gripper(self):
        service     =   rospy.Service(
                        "GripperCmd",
                        _gripper.gripper,
                        self.gripper_srv
                        )
        return service

    def approach(self):
        service     =   rospy.Service(
                        "ApproachCmd",
                        _approach.approach,
                        self._approach
                        )
        return service

    def ServoToPose(self):
        service     =   rospy.Service(
                        "ServoToPoseCmd",
                        _servotoPose.servotoPose,
                        self._servo_to_pose
                        )
        return service

    def retract(self):
        service     =   rospy.Service("RetractCmd", _retract.retract, self._retract)
        return service

    @staticmethod
    def _loginfo(NodeName, message):
        # type: (str, str) -> None
        rospy.loginfo(f'CommandServerLog({NodeName}):  {message}')



def main():
    tt = CommandServer() 
    srv1 = tt.gripper()
    srv2 = tt.approach()
    srv3 = tt.ServoToPose()
    srv4 = tt.retract()
    rospy.spin()

main()