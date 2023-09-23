#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <geometry_msgs/PoseWithCovariance.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>

#include "pick_and_place/approach.h"
#include "pick_and_place/gripper.h"
#include "pick_and_place/retract.h"
#include "pick_and_place/servotoPose.h"


using namespace BT;

class gripperOpen : public BT::SyncActionNode{
    private:
    ros::NodeHandle _nh;
    public:
        gripperOpen(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){
            this->_nh = nh;
        }

        BT::NodeStatus tick() override{
            // Client code to open the gripper and recieve a response
            ros::ServiceClient client = this->_nh.serviceClient<pick_and_place::gripper>(std::string("GripperCmd"));
            pick_and_place::gripper srv_call;
            srv_call.request.gripper_cmd = std::string("Open");
            ros::service::waitForService("GripperCmd", ros::Duration(5));
            if (client.call(srv_call)) {
                ROS_INFO("Server call successful! Response was %d", srv_call.response.gripper_reply);
                return BT::NodeStatus::SUCCESS;
            } else {
                ROS_ERROR("Failed to call 'checks' service");
                return BT::NodeStatus::FAILURE;
            }
        }

        static BT::PortsList providedPorts(){
            return{};
        }
};

class gripperClose : public BT::SyncActionNode{
    private:
    ros::NodeHandle _nh;
    public:
        gripperClose(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){
            this->_nh = nh;
        }

        BT::NodeStatus tick() override{
            // Client code to open the gripper and recieve a response
            ros::ServiceClient client = this->_nh.serviceClient<pick_and_place::gripper>(std::string("GripperCmd"));
            pick_and_place::gripper srv_call;
            srv_call.request.gripper_cmd = std::string("Close");
            ros::service::waitForService("GripperCmd", ros::Duration(5));
            if (client.call(srv_call)) {
                ROS_INFO("Server call successful! Response was %d", srv_call.response.gripper_reply);
                return BT::NodeStatus::SUCCESS;
            } else {
                ROS_ERROR("Failed to call 'checks' service");
                return BT::NodeStatus::FAILURE;
            }
        }

        static BT::PortsList providedPorts(){
            return{};
        }
};

class approach : public BT::SyncActionNode{
    private:
        ros::NodeHandle _nh;

    public:
        approach(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){
            this->_nh = nh;
        }

        BT::NodeStatus tick() override{
            // Read from blackboard
            geometry_msgs::Pose target_pose;
            BT::TreeNode::getInput(std::string("approachPose"), target_pose);
            // Call service here
            ros::ServiceClient client = this->_nh.serviceClient<pick_and_place::approach>(std::string("ApproachCmd"));
            pick_and_place::approach srv_call;
            srv_call.request.approach_pose = target_pose;
            ros::service::waitForService("ApproachCmd", ros::Duration(5));

            // Write to blackboard?
            if(!client.call(srv_call)){
                ROS_INFO("[MoveitCartesianPathPlanning] Error when executing plan {move_group->execute(plan)}");
                return BT::NodeStatus::SUCCESS;
            }
            else{
                ROS_ERROR("[MoveitCartesianPathPlanning] Error when executing plan {move_group->execute(plan)}");
                return BT::NodeStatus::FAILURE;
            }
        }

        static BT::PortsList providedPorts(){
            return{};
        }
};

class ServoToPose : public BT::SyncActionNode{
    private:
        ros::NodeHandle _nh;

    public:
        ServoToPose(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){
            this->_nh = nh;
        }

        BT::NodeStatus tick() override{
            // Read from blackboard
            geometry_msgs::Pose target_pose;
            BT::TreeNode::getInput(std::string("ServoToPose"), target_pose);

            // Call service here
            ros::ServiceClient client = this->_nh.serviceClient<pick_and_place::servotoPose>(std::string("ServoToPoseCmd"));

            pick_and_place::servotoPose srv_call;

            srv_call.request.servo_to_pose = target_pose;

            ros::service::waitForService("ServoToPoseCmd", ros::Duration(5));

            // Write to blackboard?
            if(!client.call(srv_call)){
                ROS_INFO("[MoveitCartesianPathPlanning] Error when executing plan {move_group->execute(plan)}");
                return BT::NodeStatus::SUCCESS;
            }
            else{
                ROS_ERROR("[MoveitCartesianPathPlanning] Error when executing plan {move_group->execute(plan)}");
                return BT::NodeStatus::FAILURE;
            }
        }

        static BT::PortsList providedPorts(){
            return{};
        }
};

class retract : public BT::SyncActionNode{
    private:
        ros::NodeHandle _nh;

    public:
        retract(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){
            this->_nh = nh;
        }


        BT::NodeStatus tick() override{
            // Call service here
            ros::ServiceClient client = this->_nh.serviceClient<pick_and_place::retract>(std::string("RetractCmd"));

            pick_and_place::retract srv_call;

            srv_call.request.retract_cmd = true;

            ros::service::waitForService("RetractCmd", ros::Duration(5));

            if(!client.call(srv_call)){
                ROS_INFO("[MoveitCartesianPathPlanning] Error when executing plan {move_group->execute(plan)}");
                return BT::NodeStatus::SUCCESS;
            }
            else{
                ROS_ERROR("[MoveitCartesianPathPlanning] Error when executing plan {move_group->execute(plan)}");
                return BT::NodeStatus::FAILURE;
            }
        }

        static BT::PortsList providedPorts(){
            return{};
        }
};

int main(){
    std::cout << "Exec Test" << std::endl;
    return 0;
}