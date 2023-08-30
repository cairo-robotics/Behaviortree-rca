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
            std::cout << "Strating picking operation ... " << std::endl;
            planBase->Pick();
            return BT::NodeStatus::SUCCESS;
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

    class DetectArucoMarkers : public BT::SyncActionNode{
        public:
            DetectArucoMarkers(const std::string& name, const BT::NodeConfiguration& config)            
            : BT::SyncActionNode(name, config)
            { }

            BT::NodeStatus tick() override{
                callback::aruco_class myArucos;
                for(int i = 0; i < myArucos.aruco_array.markers.size(); i++){
                    ArucoPoses.push_back(myArucos.aruco_array.markers[i].pose);
                    Arucolabels.push_back(myArucos.aruco_array.markers[i].id);
                }
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

}

static const char* xml_text = R"(
 <root BTCPP_format="3">
     <BehaviorTree>
        <Sequence>
            <Pick/>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

int main(int argc, char **argv){
    ros::init(argc, argv, "BehaviorTreeNode");
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    BehaviorTreeFactory factory;
    my_planning::MyPlanningClass planning;
    PnPNamespace::setPlannerforNamespace(&planning);
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.38;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.65;
    PnPNamespace::setGoalforNamespace(target_pose1);
    factory.registerNodeType<PnPNamespace::gotoGoal>("gotoGoal");
    factory.registerNodeType<PnPNamespace::Pick>("Pick");
    auto tree = factory.createTreeFromText(xml_text);
    tree.tickRoot();
    spinner.stop();
    return 0;
}