#include <aruco_msgs/MarkerArray.h>


namespace callback{
    class aruco_class{
        public:
        std::vector<int> labels;
        std::vector<geometry_msgs::PoseWithCovariance> Poses;
        aruco_msgs::MarkerArray aruco_array;
        ros::NodeHandle nh_;

        void DetectArucoPlacesCallback(const aruco_msgs::MarkerArray msg){
            aruco_array = msg;
        } 

        aruco_class(){
            int argc = 0;
            char** argv = NULL; 
            ros::init(argc, argv,"custom_interfacing");
        }
    };

    aruco_msgs::MarkerArray call_arucoclass(){
        aruco_class ar;
        int i = 0;
        ros::Subscriber subscriber = ar.nh_.subscribe("/aruco_topic",1, &aruco_class::DetectArucoPlacesCallback, &ar);
        while(i < 10){
            ros::spinOnce();
        }
        return ar.aruco_array;
    }
}
