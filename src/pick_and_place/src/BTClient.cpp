#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include "jsoncpp/json/json.h"


#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "pick_and_place/approach.h"
#include "pick_and_place/gripper.h"
#include "pick_and_place/retract.h"
#include "pick_and_place/servotoPose.h"

using namespace BT;


/**
 * splitString: Splits string into vector of <x, y, z, q1, q2, q3, q4>
 * 
 * @param input     String recieved from blackboard carrying reference pose.
 * @param delimiter Delimiter for input string.
 * @return Vector of floats having 7 elements of pose
 */

std::vector<float> splitString(const std::string input, const char& delimiter) {

    std::vector<float> elements;
    std::stringstream stream(input);
    std::string element;

    while (getline(stream, element, delimiter)) {
        elements.push_back(stof(element));
    }

    return elements;
}

/**
 * createMsg: Creates a ros pose message from the vector of 7 elements
 * 
 * @param msg       Pointer to ros message that holds values from the vector.
 * @param poseVal   Vector obtained from convering Blackboard string to vector.
 * @return Void return type
 */

void createMsg (geometry_msgs::Pose* msg, const std::vector<float> poseVal){
    msg->position.x = poseVal[0];
    msg->position.y = poseVal[1];
    msg->position.z = poseVal[2];
    msg->orientation.x = poseVal[3];
    msg->orientation.y = poseVal[4];
    msg->orientation.z = poseVal[5];
    msg->orientation.w = poseVal[6];
}

/**
 * Creates a string from a Ros pose message to be stored in blackboard
 * 
 * @param msg       ROS pose message to be written to blackboard
 * @param delimiter Delimiter string for the blackboard string
 * @return String that gets written to the blackboard
 */

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

/**
 * Creates a pose from a json file stored on the filesystem.
 * 
 * @param fileName  FileName to be read, ends with .json.
 * @return tf::Transform type object that has static transform.
 */

tf::Transform poseFromJSON(std::string fileName){
    Json::Value root;
    std::ifstream ifs;
    ifs.open(&fileName[0]);
    
    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    JSONCPP_STRING errs;
    if(!Json::parseFromStream(builder, ifs, &root, &errs)){
        std::cout << errs << std::endl;
    }
    std::string names("CalibrationPose");
    return tf::Transform(tf::Matrix3x3(tf::Quaternion(  root[names][3].asDouble(),
                                                        root[names][4].asDouble(),
                                                        root[names][5].asDouble(),
                                                        root[names][6].asDouble())),
                                       tf::Vector3(     root[names][0].asDouble(),
                                                        root[names][1].asDouble(),
                                                        root[names][2].asDouble()));
}

/**
 *  GripperOpen class that inherits the Behaviortree.CPP Synchronous Action Node class,
 *  and creates an Action node to open the sawyer gripper
 */


class gripperOpen : public BT::SyncActionNode{
    private:
        ros::NodeHandle _nh;
    public:
        gripperOpen(const std::string& name, const BT::NodeConfiguration& config,ros::NodeHandle nh)
        : BT::SyncActionNode(name, config), _nh(nh){}

        /**
         * GripperOpen class: Tick function that overrides a virtual function to call a service that opens the gripper.
         * 
         * @return Node status boolean for the Behaviortree.CPP node
         */

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

/**
 *  GripperClose class that inherits the Behaviortree.CPP Synchronous Action Node class,
 *  and creates an Action node to close the sawyer gripper
 */

class gripperClose : public BT::SyncActionNode{
    private:
        ros::NodeHandle _nh;
    public:
        gripperClose(const std::string& name, const BT::NodeConfiguration& config,ros::NodeHandle nh)
        : BT::SyncActionNode(name, config), _nh(nh){}

        /**
         * GripperClose class: Tick function that overrides a virtual function to call a ros-service that closes the gripper.
         * 
         * @return Node status boolean for the Behaviortree.CPP node
         */

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

/**
 *  Approach class that inherits the Behaviortree.CPP Synchronous Action Node class,
 *  and creates an Action node to approach the Ket or to approach the hole in which Ket needs to be placed.
 */

class approach : public BT::SyncActionNode{
    private:
        ros::NodeHandle _nh;
        geometry_msgs::Pose _targetPose;

    public:
        approach(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh)
        : BT::SyncActionNode(name, config), _nh(nh){}

        /**
         * Approach class: Tick function that overrides a virtual function to call a ros-service
         * that servos to a prestored pose in blackboard named as "approachPose".
         * 
         * @return Node status boolean for the Behaviortree.CPP node
         * @throw  Throws Runtime error if blackboard value is empty
         */

        BT::NodeStatus tick() override{
            // Initialise service
            ros::ServiceClient client = this->_nh.serviceClient<pick_and_place::approach>(std::string("ApproachCmd"));

            std::string msg;
            BT::TreeNode::getInput(std::string("approachPose"), msg);

            // Check if expected is valid. If not, throw its error
            if (msg == ""){
                throw BT::RuntimeError("missing required input [message]");
            }

            char delimiter = ';';
            std::vector<float> poseVal = splitString(msg, delimiter);
            createMsg(&this->_targetPose, poseVal);

            pick_and_place::approach srv_call;
            srv_call.request.approach_pose = this->_targetPose;
            ros::service::waitForService("ApproachCmd", ros::Duration(5));

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

/**
 *  ServoToPose class that inherits the Behaviortree.CPP Synchronous Action Node class,
 *  and creates an Action node. To be called to move the arm around freely to another pose. 
 *  Does not hold any precondition, of any sort of pose extraction.
 */

class ServoToPose : public BT::SyncActionNode{
    private:
        ros::NodeHandle _nh;
        geometry_msgs::Pose _targetPose;
    public:
        ServoToPose(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh)
        : BT::SyncActionNode(name, config), _nh(nh){}

        /**
         * ServoToPose class: Tick function that overrides a virtual function to call a ros-service
         * that servos to a prestored pose in blackboard named as "ServoToPose".
         * 
         * @return Node status boolean for the Behaviortree.CPP node
         * @throw  Throws Runtime error if blackboard value is empty
         */

        BT::NodeStatus tick() override{
            // Read from blackboard
            std::string msg;
            BT::TreeNode::getInput("ServoToPose", msg);

            // Check if expected is valid. If not, throw its error
            if (msg == ""){
                throw BT::RuntimeError("missing required input [message]");
            }
            std::cout << "ServoToPose_msg: " << msg << std::endl; 
            // use the method value() to extract the valid message.
            char delimiter = ';';
            std::vector<float> poseVal = splitString(msg, delimiter);
            createMsg(&this->_targetPose, poseVal);
            // Call service here
            ros::ServiceClient client = this->_nh.serviceClient<pick_and_place::servotoPose>(std::string("ServoToPoseCmd"));

            pick_and_place::servotoPose srv_call;

            srv_call.request.servo_to_pose = this->_targetPose;
            std::cout << this->_targetPose;
            ros::service::waitForService("ServoToPoseCmd", ros::Duration(5));

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


/**
 *  Retract class that inherits the Behaviortree.CPP Synchronous Action Node class,
 *  and creates an Action node to approach one of the two preprogrammed positions to head to after picking up the ket.
 *  To be called after holding the ket and retracting to a pre-defined position and retracting after placing the ket in the hole.
 */

class retract : public BT::SyncActionNode{
    // TODO: If we have two positions to retract to, how to choose one? Ideally there should be an input from the client that server parses and executes the same
    private:
        ros::NodeHandle _nh;

    public:
        retract(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh)
        : BT::SyncActionNode(name, config), _nh(nh){}

        /**
         * Retract class: Tick function that overrides a virtual function to call a ros-service
         * that servos to a prestored pose 
         * 
         * @return Node status boolean for the Behaviortree.CPP node
         * @throw  Throws Runtime error if blackboard value is empty
         */

        BT::NodeStatus tick() override{
            // Call service here
            std::string msg;
            ros::ServiceClient client = this->_nh.serviceClient<pick_and_place::retract>(std::string("RetractCmd"));
            BT::TreeNode::getInput("retractCmd", msg);
            pick_and_place::retract srv_call;
            
            std::cout << "msg: " << (msg == std::string("True")) << "\n";
            srv_call.request.retract_cmd = (msg == std::string("True"));
            
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
            return{InputPort<std::string>("retractCmd")};
        }
};

/**
 *  VisualFeedback class that inherits the Behaviortree.CPP Synchronous Action Node class,
 *  and creates an Action node to read the topic of DL node to detect where to place the Ket, 
 *  also writes it to the blackboard to servo to eventually.
 */

class visualFeedback : public BT::SyncActionNode{
    private:
        ros::NodeHandle _nh;
        geometry_msgs::PoseStamped _desiredPosition;

    public:
        visualFeedback(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh)
        : BT::SyncActionNode(name, config), _nh(nh){
            this->_desiredPosition.header.seq = -1; // Default setting to check weather the message has been modified by callback or not;
        }

        void visualFeedbackCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
            this->_desiredPosition.pose     = msg->pose;
            this->_desiredPosition.header    = msg->header; 
            std::string poseString = createString(&this->_desiredPosition.pose, std::string(";"));
        }

        /**
         * VisualFeedback class: Tick function spins the node once to collect pose from
         * the deeplearning node that detects the location at which the ket is to be placed.
         * 
         * @return Node status boolean for the Behaviortree.CPP node
         * @throw  Throws Runtime error if blackboard value is empty
         */

        BT::NodeStatus tick() override{
            // Call service here
            std::cout << "This should execute first \n";
            ros::Rate r(10);
            ros::Subscriber sub = this->_nh.subscribe("/visionFeedback/MeanValue", 1, &visualFeedback::visualFeedbackCallback, this);
            for(int i = 0 ; i < 10; i++){
                ros::spinOnce();
                r.sleep();
            }
     
            if(!(this->_desiredPosition.header.seq == -1)){
                // Convert pose in camera frame to gripper frame and store in blackboard
                tf::Transform c_T_K, g_T_c;
                tf::poseMsgToTF(this->_desiredPosition.pose, c_T_K);
                g_T_c = poseFromJSON("src/pick_and_place/src/GripperToCameraTransform.json");
                tf::Transform finalGoal = (tf::Transform(c_T_K.inverse()*g_T_c));
                geometry_msgs::Pose finalGoalPose;
                tf::poseTFToMsg(finalGoal, finalGoalPose);

                // Write variable to the blackboard
                std::string poseString = createString(&finalGoalPose, std::string(";"));
                BT::TreeNode::setOutput(std::string("ServoToPose"), poseString);
                ROS_INFO("[VisualFeedbackNode] Executed Successfully ");
                return BT::NodeStatus::SUCCESS;
            }
            else{
                ROS_INFO("[VisualFeedbackNode] Error in recieving message check server {pick pose determining server}");
                return BT::NodeStatus::SUCCESS;
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

static const char* testVisualFeedback = R"(
 <root BTCPP_format="3">
    <BehaviorTree ID="DemoTry">
        <Sequence>
            <gripperOpen/>
            <retract retractCmd="True"/>
            <visualFeedback ServoToPose="{ServoToPose}"/>
            <ServoToPose ServoToPose="{ServoToPose}"/>
            <gripperClose/>
            <retract retractCmd="False"/>
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

    BT::NodeBuilder visualFeedback_node =
    [nh_](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<visualFeedback>(name, config, nh_);
    };

    factory.registerBuilder<gripperClose>("gripperClose", gripper_close);
    factory.registerBuilder<gripperOpen>("gripperOpen", gripper_open);
    factory.registerBuilder<ServoToPose>("ServoToPose", servo_to_pose_node);
    factory.registerBuilder<retract>("retract", retract_node);
    factory.registerBuilder<visualFeedback>("visualFeedback", visualFeedback_node);

    auto tree = factory.createTreeFromText(::testVisualFeedback);

    tree.tickRootWhileRunning();
    spinner.stop();
    return 0;
}