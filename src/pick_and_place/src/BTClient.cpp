#include <iostream>
#include <string>
#include <vector>
#include <sstream>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>

#include "pick_and_place/approach.h"
#include "pick_and_place/gripper.h"
#include "pick_and_place/retract.h"
#include "pick_and_place/servotoPose.h"


using namespace BT;

std::vector<float> splitString(const std::string input, const char& delimiter) {

    std::vector<float> elements;
    std::stringstream stream(input);
    std::string element;

    while (getline(stream, element, delimiter)) {
        elements.push_back(stof(element));
    }

    return elements;
}

void createMsg (geometry_msgs::Pose* msg, const std::vector<float> poseVal){
    msg->position.x = poseVal[0];
    msg->position.y = poseVal[1];
    msg->position.z = poseVal[2];
    msg->orientation.x = poseVal[3];
    msg->orientation.y = poseVal[4];
    msg->orientation.z = poseVal[5];
    msg->orientation.w = poseVal[6];
}

std::string createString(geometry_msgs::Pose* msg, std::string delimiter){
    std::string ans;
    ans = std::to_string(msg->position.x) + delimiter;
    ans += std::to_string(msg->position.y) + delimiter;
    ans += std::to_string(msg->position.z) + delimiter;
    ans += std::to_string(msg->orientation.x) + delimiter;
    ans += std::to_string(msg->orientation.y) + delimiter;
    ans += std::to_string(msg->orientation.z) + delimiter;
    ans += std::to_string(msg->orientation.w) + delimiter;
    return ans;
}

class gripperOpen : public BT::SyncActionNode{
    private:
        ros::NodeHandle _nh;
    public:
        gripperOpen(const std::string& name, const BT::NodeConfiguration& config,ros::NodeHandle nh)
        : BT::SyncActionNode(name, config), _nh(nh){}

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
            return {};
        }

};

class gripperClose : public BT::SyncActionNode{
    private:
        ros::NodeHandle _nh;
    public:
        gripperClose(const std::string& name, const BT::NodeConfiguration& config,ros::NodeHandle nh)
        : BT::SyncActionNode(name, config), _nh(nh){}

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
            return {};
        }
};

class approach : public BT::SyncActionNode{
    private:
        ros::NodeHandle _nh;
        geometry_msgs::Pose _targetPose;

    public:
        approach(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh)
        : BT::SyncActionNode(name, config), _nh(nh){}

        BT::NodeStatus tick() override{
            // Call service here
            ros::ServiceClient client = this->_nh.serviceClient<pick_and_place::approach>(std::string("ApproachCmd"));

            std::string msg;
            BT::TreeNode::getInput(std::string("approachPose"), msg);

            // Check if expected is valid. If not, throw its error
            if (msg == ""){
                throw BT::RuntimeError("missing required input [message]");
            }

            // use the method value() to extract the valid message.
            char delimiter = ';';
            std::vector<float> poseVal = splitString(msg, delimiter);
            createMsg(&this->_targetPose, poseVal);

            pick_and_place::approach srv_call;
            srv_call.request.approach_pose = this->_targetPose;
            ros::service::waitForService("ApproachCmd", ros::Duration(5));

            // Write to blackboard?
            if (client.call(srv_call)) {
                ROS_INFO("Server call successful! Response was %d", srv_call.response.approach_reply);
                return BT::NodeStatus::SUCCESS;
            } else {
                ROS_ERROR("Failed to call 'checks' service");
                return BT::NodeStatus::FAILURE;
            }
        }

        static BT::PortsList providedPorts(){
            return {InputPort<std::string>("approachPose")};
        }
};

class ServoToPose : public BT::SyncActionNode{
    private:
        ros::NodeHandle _nh;
        geometry_msgs::Pose _targetPose;
    public:
        ServoToPose(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh)
        : BT::SyncActionNode(name, config), _nh(nh){}

        BT::NodeStatus tick() override{
            // Read from blackboard
            std::string msg;
            BT::TreeNode::getInput(std::string("ServoToPose"), msg);

            // Check if expected is valid. If not, throw its error
            if (msg == ""){
                throw BT::RuntimeError("missing required input [message]");
            }

            // use the method value() to extract the valid message.
            char delimiter = ';';
            std::vector<float> poseVal = splitString(msg, delimiter);
            createMsg(&this->_targetPose, poseVal);
            // Call service here
            ros::ServiceClient client = this->_nh.serviceClient<pick_and_place::servotoPose>(std::string("ServoToPoseCmd"));

            pick_and_place::servotoPose srv_call;

            srv_call.request.servo_to_pose = this->_targetPose;

            ros::service::waitForService("ServoToPoseCmd", ros::Duration(5));

            // Write to blackboard?
            if (client.call(srv_call)) {
                ROS_INFO("Server call successful! Response was %d", srv_call.response.servotopose_reply);
                return BT::NodeStatus::SUCCESS;
            } else {
                ROS_ERROR("Failed to call 'checks' service");
                return BT::NodeStatus::FAILURE;
            }
        }

        static BT::PortsList providedPorts(){
            return{InputPort<std::string>("ServoToPose")};
        }
};

class retract : public BT::SyncActionNode{
    // TODO: If we have two positions to retract to, how to choose one? Ideally there should be an input from the client that server parses and executes the same
    private:
        ros::NodeHandle _nh;

    public:
        retract(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh)
        : BT::SyncActionNode(name, config), _nh(nh){}

        BT::NodeStatus tick() override{
            // Call service here
            ros::ServiceClient client = this->_nh.serviceClient<pick_and_place::retract>(std::string("RetractCmd"));

            pick_and_place::retract srv_call;

            srv_call.request.retract_cmd = true;

            ros::service::waitForService("RetractCmd", ros::Duration(5));

            if (client.call(srv_call)) {
                ROS_INFO("Server call successful! Response was %d", srv_call.response.retract_reply);
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

class visualFeedback : public BT::SyncActionNode{
    private:
        ros::NodeHandle _nh;
        geometry_msgs::PoseStamped _desiredPosition;

    public:
        visualFeedback(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh)
        : BT::SyncActionNode(name, config), _nh(nh){
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "visualFeedbackReader");
            this->_desiredPosition.header.seq = -1; // Default setting to check weather the message has been modified by callback or not;
        }

        void visualFeedbackCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
            this->_desiredPosition.pose     = msg->pose;
            this->_desiredPosition.header    = msg->header; 
        }

        BT::NodeStatus tick() override{
            // Call service here
            ros::Subscriber sub = this->_nh.subscribe("/visionFeedback/MeanValue", 1, &visualFeedback::visualFeedbackCallback, this);

            ros::spinOnce();

            if(!(this->_desiredPosition.header.seq == -1)){
                // Write variable to the blackboard
                std::string poseString = createString(&this->_desiredPosition.pose, std::string(";"));
                setOutput("ServoToPose", poseString);
                ROS_INFO("[MoveitCartesianPathPlanning] Error when executing plan {move_group->execute(plan)}");
                return BT::NodeStatus::SUCCESS;
            }
            else{
                ROS_ERROR("[MoveitCartesianPathPlanning] Error when executing plan {move_group->execute(plan)}");
                return BT::NodeStatus::FAILURE;
            }
        }

        static BT::PortsList providedPorts(){
            return{OutputPort<std::string>("ServoToPose")};
        }
};

static const char* pickTree = R"(
 <root BTCPP_format="3">
     <BehaviorTree ID="DemoTry">
        <Sequence>
            <gripperOpen/>
            <RetryUntilSuccessful num_attempts="5">
                <approach/>
            </RetryUntilSuccessful>
            <RetryUntilSuccessful num_attempts="5">
                <ServoToPose/>
            </RetryUntilSuccessful>
            <gripperClose/>
            <RetryUntilSuccessful num_attempts="5">
                <retract/>
            </RetryUntilSuccessful>
        </Sequence>
     </BehaviorTree ID="DemoTry">
 </root>
)";

static const char* visualServoTree = R"(
 <root BTCPP_format="3">
      <BehaviorTree ID="DemoTry">
        <Sequence>
            <RetryUntilSuccessful num_attempts="5">
                <Sequence>
                    <visualFeedback/>
                    <approach/>
                </Sequence>
            </RetryUntilSuccessful>
            <RetryUntilSuccessful num_attempts="5">
                <ServoToPose/>
            </RetryUntilSuccessful>
            <gripperOpen/>
            <RetryUntilSuccessful num_attempts="5">
                <retract/>
            </RetryUntilSuccessful>
        </Sequence>
      </BehaviorTree ID="DemoTry">
 </root>
)";

static const char* testTree = R"(
 <root BTCPP_format="3">
    <BehaviorTree ID="DemoTry">
        <Sequence>
            <gripperClose/>     
            <Sequence>   
                <retract/>
            </Sequence>
            <gripperOpen/>
        </Sequence>
    </BehaviorTree ID="DemoTry">
 </root>
)";

int main(int argc, char **argv){
    ros::init(argc, argv, "BehaviorTreeNode");
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    BehaviorTreeFactory factory;

    BT::NodeBuilder gripper_close =
    [nh_](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<gripperClose>(name, config, nh_);
    };

    BT::NodeBuilder gripper_open =
    [nh_](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<gripperOpen>(name, config, nh_);
    };

    BT::NodeBuilder approach_node =
    [nh_](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<approach>(name, config, nh_);
    };

    BT::NodeBuilder servo_to_pose_node =
    [nh_](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<ServoToPose>(name, config, nh_);
    };

    BT::NodeBuilder retract_node =
    [nh_](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<retract>(name, config, nh_);
    };

    factory.registerBuilder<gripperClose>("gripperClose", gripper_close);
    factory.registerBuilder<gripperOpen>("gripperOpen", gripper_open);
    factory.registerBuilder<retract>("retract", retract_node);

    auto tree = factory.createTreeFromText(::testTree);

    tree.tickRootWhileRunning();
    spinner.stop();
    std::cout << "Exec Test" << std::endl;
    return 0;
}