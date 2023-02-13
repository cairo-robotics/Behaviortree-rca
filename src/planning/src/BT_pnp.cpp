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

class Actions{
        std::vector<int> Arucolabels;
        std::vector<geometry_msgs::PoseWithCovariance> ArucoPoses;

    public:
        BT::NodeStatus Pick(my_planning::MyPlanningClass* planBase){
            planBase->Pick();
        }
        BT::NodeStatus gotoGoal(geometry_msgs::Pose& goalPose, my_planning::MyPlanningClass* planBase){
            planBase->goToPoseGoal(goalPose);
        }

        BT::NodeStatus Place(my_planning::MyPlanningClass* planBase){
            planBase->Place();
        }

        BT::NodeStatus DetectArucoMarkers(){
            callback::aruco_class myArucos;
            for(int i = 0; i < myArucos.aruco_array.markers.size(); i++){
                ArucoPoses.push_back(myArucos.aruco_array.markers[i].pose);
                Arucolabels.push_back(myArucos.aruco_array.markers[i].id);
            }
        }
};

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

int main(){
    // BehaviorTreeFactory factory;
    // factory.registerNodeType<gotoGoal>("gotoGoal");
    // factory.registerSimpleAction("Pick", std::bind(Pick,this));
}