#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

namespace my_planning
{
    class MyPlanningClass
    {
        public:
            MyPlanningClass(): move_group(PLANNING_GROUP)
            {
                target_pose1.orientation.w = 1.0;
                target_pose1.position.x = 0.38;
                target_pose1.position.y = -0.2;
                target_pose1.position.z = 0.65;

                this->move_group.allowReplanning(true);
                this->move_group.setNumPlanningAttempts(10);
            }

            void goToPoseGoal();
            void goToPoseGoal(geometry_msgs::Pose &pose);
            void goToJointState();
            void cartesianPath();
            void resetValues();
            void addObjects();
            void makeBox(std::string blk_name, double *pose);
            void removeObjects();
            void closedGripper(trajectory_msgs::JointTrajectory& posture);
            void openGripper(trajectory_msgs::JointTrajectory& posture);
            void Pick();
            void Place();

        private:
            const std::string PLANNING_GROUP = "right_arm";
            const double tau = 2 * M_PI;
            moveit::planning_interface::MoveGroupInterface move_group;
            moveit::planning_interface::PlanningSceneInterface virtual_world;
            const robot_state::JointModelGroup* joint_model_group;
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            geometry_msgs::Pose target_pose1;
            geometry_msgs::Pose pick1;
    };
}