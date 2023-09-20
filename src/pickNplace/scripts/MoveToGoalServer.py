import intera_interface
import rospy
import actionlib
from pickNplace.msg import desired_goal, response


class GoalActionServer:
    def __init__(self) -> None:
        self.ReachGoalActionServer = actionlib.SimpleActionServer("PickAction", desired_goal, self.reachGoal)
    
    def start(self):
        self._loginfo('ActionServer Started')
        self.ReachGoalActionServer.start()
    
    def stop(self):
        self._loginfo('ActionServer Stopped')
    
    def reachGoal(self, goal):
        limb1 = intera_interface.Limb("right")
        
        try:
            limb1.move_to_neutral()
            self._loginfo('Goto neutral action succeeded')
            result = response()
            result.success = True
            self.ReachGoalActionServer.set_aborted(result)

        except:
            self._loginfo('Goto neutral action aborted')
            result = response()
            result.success = False
            self.ReachGoalActionServer.set_aborted(result)
        
        try:
            
            limb1.move_to_joint_positions(goal)
            self._loginfo('Goto Goal action succeeded')
            result = response()
            result.success = True
            self.ReachGoalActionServer.set_aborted(result)
        except:
            self._loginfo('Goto goal action aborted')
            result = response()
            result.success = False
            self.ReachGoalActionServer.set_aborted(result)

    @staticmethod
    def _loginfo(message):
        # type: (str) -> None
        rospy.loginfo('GotoGoalActionServer ({}) {}'.format('GotoGoalActionServer', message))

def startMoveActionLib():
    rospy.init_node("MoveToGoalServer")
    MoveToGoalServer = GoalActionServer()
    MoveToGoalServer.start()
    
    def stopMoveActionLib():
        MoveToGoalServer.stop()
    
    rospy.on_shutdown(stopMoveActionLib)
    
    rospy.spin()

if __name__ == '__main__':
    startMoveActionLib()