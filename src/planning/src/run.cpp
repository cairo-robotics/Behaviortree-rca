#include <planning.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "custom_interfacing");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    if(argc != 2)
    {
        ROS_INFO(" ");
        ROS_INFO("\tUsage:");
        ROS_INFO(" ");
        ROS_INFO("\trosrun planning run  n");
        return 1;
    }

    my_planning::MyPlanningClass my_planning_;

    int selection = atoi(argv[1]);
    switch(selection)
    {
        my_planning_.resetValues();

        case 1:
            my_planning_.goToPoseGoal();
            break;
        case 2:
            my_planning_.goToJointState();
            break;
        case 3:
            my_planning_.cartesianPath();
            break;
        case 4:
            my_planning_.addObjects();
            break;
        case 5:
            my_planning_.removeObjects();
    }

    spinner.stop();
    return 0;
}

