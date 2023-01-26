#include <planning.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>


using namespace BT;


class Pick : public BT::SyncActionNode{
    public:
        my_planning::MyPlanningClass* planBase;

        static BT::PortsList providedPorts(){
            BT::PortsList ports = {};
            return ports;
        }

        Pick(const std::string& name, const BT::NodeConfiguration& config, my_planning::MyPlanningClass& planning) 
            : BT::SyncActionNode(name, config), planBase(planning)
        {}

        BT::NodeStatus tick() override{
            planBase->pick();
        }
        virtual BT::NodeStatus on_success()
        {
            return BT::NodeStatus::SUCCESS;
        }

        /**
        * @brief Function to perform some user-defined operation whe the action is aborted.
        * @return BT::NodeStatus Returns FAILURE by default, user may override return another value
        */
        virtual BT::NodeStatus on_aborted()
        {
            return BT::NodeStatus::FAILURE;
        }

        /**
        * @brief Function to perform some user-defined operation when the action is cancelled.
        * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
        */
        virtual BT::NodeStatus on_cancelled()
        {
            return BT::NodeStatus::SUCCESS;
        }
}

class gotoGoal : public BT::SyncActionNode{
    public:
        geometry_msgs::Pose* goalPose;
        my_planning::MyPlanningClass* planBase;

        gotoGoal(const std::string& name, const BT::NodeConfiguration& config, geometry_msgs::Pose& pose, my_planning::MyPlanningClass& planning)
        :   BT::SyncActionNode(name, config), goalPose(pose){ }

        BT::NodeStatus tick() override{
            planBase->goToPoseGoal(goalPose);
        }
        virtual BT::NodeStatus on_success()
        {
            return BT::NodeStatus::SUCCESS;
        }

        /**
        * @brief Function to perform some user-defined operation whe the action is aborted.
        * @return BT::NodeStatus Returns FAILURE by default, user may override return another value
        */
        virtual BT::NodeStatus on_aborted()
        {
            return BT::NodeStatus::FAILURE;
        }

        /**
        * @brief Function to perform some user-defined operation when the action is cancelled.
        * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
        */
        virtual BT::NodeStatus on_cancelled()
        {
            return BT::NodeStatus::SUCCESS;
        }
}


class 