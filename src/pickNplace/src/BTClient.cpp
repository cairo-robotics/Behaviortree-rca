#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <geometry_msgs/PoseWithCovariance.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>

#include "pickNplace/srv/approach.h"
#include "pickNplace/srv/gripper.h"
#include "pickNplace/srv/retract.h"
#include "pickNplace/srv/servotoPose.h"


using namespace BT;

class gripperOpen : public BT::SyncActionNode{
    private:
    ros::NodeHandle* _nh;
    public:
        gripperOpen(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){
            this->_nh = nh;
        }

        BT::NodeStatus tick() override{
            // Client code to open the gripper and recieve a response
            ros::ServiceClient client = this->_nh.serviceClient<pickNplace::gripper>(std::string("GripperCmd"));
            pickNplace::GripperCmd srv_call;
            srv_call.request.gripper_cmd = std::string("Open");
            ros::service::waitForService("GripperCmd", ros::Duration(5));
            if (client.call(srv_call)) {
                ROS_INFO("Server call successful! Response was %d: %s", srv_call.response.success, srv_call.response.message.c_str());
            } else {
                ROS_ERROR("Failed to call 'checks' service");
            }
        }

        static BT::PortsList providedPorts(){
            return{};
        }
};

class gripperClose : public BT::SyncActionNode{
    private:
    ros::NodeHandle* _nh;
    public:
        gripperClose(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){
            this->_nh = nh;
        }

        BT::NodeStatus tick() override{
            // Client code to open the gripper and recieve a response
            ros::ServiceClient client = this->_nh.serviceClient<pickNplace::gripper>(std::string("GripperCmd"));
            pickNplace::GripperCmd srv_call;
            srv_call.request.gripper_cmd = std::string("Close");
            ros::service::waitForService("GripperCmd", ros::Duration(5));
            if (client.call(srv_call)) {
                ROS_INFO("Server call successful! Response was %d: %s", srv_call.response.success, srv_call.response.message.c_str());
            } else {
                ROS_ERROR("Failed to call 'checks' service");
            }
        }

        static BT::PortsList providedPorts(){
            return{};
        }
};

class approach : public BT::SyncActionNode{
    public:
    approach(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus tick() override{
        // Read from blackboard
        geometry_msgs::Pose target_pose;
        BT::TreeNode::getInput(std::string("approachPose"), target_pose);
        // Call service here
        ros::ServiceClient client = this->_nh.serviceClient<pickNplace::approach>(std::string("ApproachCmd"));
        pickNplace::ApproachCmd srv_call;
        srv_call.request.gripper_cmd = target_pose;
        ros::service::waitForService("ApproachCmd", ros::Duration(5));

        // Write to blackboard?
        if(!client.call(srv_call)){
            BT_ROS_INFO_STREAM("[MoveitCartesianPathPlanning] Error when executing plan {move_group->execute(plan)}");
            return BT::NodeStatus::FAILURE;
        }
        else{
            BT_ROS_INFO_STREAM("[MoveitCartesianPathPlanning] Error when executing plan {move_group->execute(plan)}");
            return BT::NodeStatus::SUCCESS;
        }
    }

    static BT::PortsList providedPorts(){
        return{};
    }
};

class ServoToPose : public BT::SyncActionNode{
    public:
    ServoToPose(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus tick() override{
        // Read from blackboard
        geometry_msgs::Pose target_pose;
        BT::TreeNode::getInput(std::string("ServoToPose"), target_pose);

        // Call service here
        ros::ServiceClient client = this->_nh.serviceClient<pickNplace::approach>(std::string("ServoToPoseCmd"));

        pickNplace::ServoToPoseCmd srv_call;

        srv_call.request.ServoToPoseCmd = target_pose;

        ros::service::waitForService("ServoToPoseCmd", ros::Duration(5));

        // Write to blackboard?
        if(!client.call(srv_call)){
            BT_ROS_INFO_STREAM("[MoveitCartesianPathPlanning] Error when executing plan {move_group->execute(plan)}");
            return BT::NodeStatus::FAILURE;
        }
        else{
            BT_ROS_INFO_STREAM("[MoveitCartesianPathPlanning] Error when executing plan {move_group->execute(plan)}");
            return BT::NodeStatus::SUCCESS;
        }
    }

    static BT::PortsList providedPorts(){
        return{};
    }
};

class retract : public BT::SyncActionNode{
    public:
    retract(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus tick() override{
        // Read from blackboard
        geometry_msgs::Pose target_pose;
        BT::TreeNode::getInput(std::string("retractPose"), target_pose);
        // Call service here
        ros::ServiceClient client = this->_nh.serviceClient<pickNplace::RetractCmd>(std::string("RetractCmd"));

        pickNplace::RetractCmd srv_call;

        srv_call.request.ServoToPoseCmd = target_pose;

        ros::service::waitForService("ServoToPoseCmd", ros::Duration(5));

        // Write to blackboard?
        if(!client.call(srv_call)){
            BT_ROS_INFO_STREAM("[MoveitCartesianPathPlanning] Error when executing plan {move_group->execute(plan)}");
            return BT::NodeStatus::FAILURE;
        }
        else{
            BT_ROS_INFO_STREAM("[MoveitCartesianPathPlanning] Error when executing plan {move_group->execute(plan)}");
            return BT::NodeStatus::SUCCESS;
        }
    }

    static BT::PortsList providedPorts(){
        return{};
    }
};
