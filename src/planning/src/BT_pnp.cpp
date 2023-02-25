#include <planning.h>
#include <aruco.h>
#include <aruco_msgs/MarkerArray.h>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <geometry_msgs/PoseWithCovariance.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>

using namespace BT;
namespace PnPNamespace{
    my_planning::MyPlanningClass* planBase;
    geometry_msgs::Pose goalPose;    
    std::vector<int> Arucolabels;
    std::vector<geometry_msgs::PoseWithCovariance> ArucoPoses;


    void setPlannerforNamespace(my_planning::MyPlanningClass* planning){
        planBase = planning;
    }

    void setGoalforNamespace(geometry_msgs::Pose goalVal){
        goalPose = goalVal;
    }

    class Pick : public BT::SyncActionNode{
        public:
        Pick(const std::string& name, const BT::NodeConfiguration& config) 
            : BT::SyncActionNode(name, config)
        {}

        BT::NodeStatus tick() override{
            planBase->Pick();
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

        static BT::PortsList providedPorts()
        {
            return{};
        }

    };

    class gotoGoal : public BT::SyncActionNode{
        public:
         gotoGoal(const std::string& name, const BT::NodeConfiguration& config) 
            : BT::SyncActionNode(name, config)
        {}

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

        static BT::PortsList providedPorts()
        {
            return{};
        }
    };


    class Place : public BT::SyncActionNode{
        public:
         Place(const std::string& name, const BT::NodeConfiguration& config) 
            : BT::SyncActionNode(name, config)
        {}

        BT::NodeStatus tick() override{
            planBase->Place();
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

        static BT::PortsList providedPorts()
        {
            return{};
        }
    };

    // class DetectArucoMarkers : public BT::SyncActionNode{
    //     public:
    //         DetectArucoMarkers(const std::string& name, const BT::NodeConfiguration& config)            
    //         : BT::SyncActionNode(name, config)
    //         { }

    //         BT::NodeStatus tick() override{
    //             callback::aruco_class myArucos;
    //             for(int i = 0; i < myArucos.aruco_array.markers.size(); i++){
    //                 ArucoPoses.push_back(myArucos.aruco_array.markers[i].pose);
    //                 Arucolabels.push_back(myArucos.aruco_array.markers[i].id);
    //             }
    //         }

    //         virtual BT::NodeStatus on_success()
    //         {
    //             return BT::NodeStatus::SUCCESS;
    //         }

    //         /**
    //         * @brief Function to perform some user-defined operation whe the action is aborted.
    //         * @return BT::NodeStatus Returns FAILURE by default, user may override return another value
    //         */
    //         virtual BT::NodeStatus on_aborted()
    //         {
    //             return BT::NodeStatus::FAILURE;
    //         }

    //         /**
    //         * @brief Function to perform some user-defined operation when the action is cancelled.
    //         * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
    //         */
    //         virtual BT::NodeStatus on_cancelled()
    //         {
    //             return BT::NodeStatus::SUCCESS;
    //         }

    //         static BT::PortsList providedPorts()
    //         {
    //             return{};
    //         }
    // };

}

// static const char* xml_text = R"(
//  <root BTCPP_format="3">
//      <BehaviorTree>
//         <Sequence>
//             <Action_A/>
//             <Action_B/>
//         </Sequence>
//      </BehaviorTree>
//  </root>
//  )";

int main(int argc, char **argv){
    ros::init(argc, argv, "custom_interfacing");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    BehaviorTreeFactory factory;
    // factory.registerNodeType<gotoGoal>("gotoGoal");
    my_planning::MyPlanningClass planning;
    PnPNamespace::setPlannerforNamespace(&planning);
    factory.registerNodeType<PnPNamespace::Pick>("Pick");
    spinner.stop();
    return 0;
}