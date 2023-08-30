#include <planning.h>

namespace my_planning
{
        void MyPlanningClass::goToPoseGoal()
        {
            this->move_group.setPoseTarget(target_pose1);
            bool success = (this->move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if(!success) //execute
                throw std::runtime_error("No plan found");

            this->move_group.move(); //blocking
        }
        
        void MyPlanningClass::goToPoseGoal(geometry_msgs::Pose &pose)
        {
            this->move_group.setPoseTarget(pose);
            ros::Duration(0.5).sleep();
            bool success = (this->move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            /*while (!success) //keep trying until a plan is found
            {
                
                success = (this->move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            }*/
            
            if(!success) //execute
                throw std::runtime_error("No plan found");

            this->move_group.move(); //blocking
        }

        void MyPlanningClass::goToJointState()
        {
            robot_state::RobotState current_state = *this->move_group.getCurrentState();
            //moveit::core::RobotStatePtr current_state = this->move_group.getCurrentState();
            std::vector<double> joint_positions;
            joint_model_group = current_state.getJointModelGroup(PLANNING_GROUP);
            current_state.copyJointGroupPositions(joint_model_group, joint_positions);
            //joint_positions = this->move_group.getCurrentJointValues();

            joint_positions[0] = -1.0;
            joint_positions[3] = 0.7;

            this->move_group.setJointValueTarget(joint_positions);
            bool success = (this->move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if(!success)
                throw std::runtime_error("No plan found");

            this->move_group.move(); //blocking
        }

        void MyPlanningClass::cartesianPath()
        {
            std::vector<geometry_msgs::Pose> waypoints; // Create waypoints from goal poses this does not take obstacles into account just a simple wrapper.
            waypoints.push_back(target_pose1);

            geometry_msgs::Pose target_pose2 = target_pose1;

            target_pose2.position.z -= 0.2;
            waypoints.push_back(target_pose2);

            target_pose2.position.y -= 0.2;
            waypoints.push_back(target_pose2);

            target_pose2.position.z += 0.2;
            target_pose2.position.y += 0.2;
            target_pose2.position.x -= 0.2;
            waypoints.push_back(target_pose2);

            this->move_group.setMaxVelocityScalingFactor(0.1);

            // We want the Cartesian path to be interpolated at a resolution of 1 cm
            // which is why we will specify 0.01 as the max step in Cartesian
            // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
            // Warning - disabling the jump threshold while operating real hardware can cause
            // large unpredictable motions of redundant joints and could be a safety issue
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = this->move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

            this->move_group.move();
            ROS_INFO_STREAM("Percentage of path followed: " << fraction);
        }

        void MyPlanningClass::resetValues()
        {
            //set the start state and operational speed
            this->move_group.setStartStateToCurrentState();
            this->move_group.setMaxVelocityScalingFactor(1.0);
        }

        void MyPlanningClass::makeBox(std::string blk_name, double *pose)
        {
            moveit_msgs::CollisionObject box;
            //set the relative frame
            box.header.frame_id = this->move_group.getPlanningFrame();
            box.id = blk_name;

            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.2;
            primitive.dimensions[1] = 0.2;
            primitive.dimensions[2] = 1.0;

            geometry_msgs::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x = pose[0];
            box_pose.position.y = pose[1];
            box_pose.position.z = pose[2];

            box.primitives.push_back(primitive);
            box.primitive_poses.push_back(box_pose);
            box.operation = box.ADD;

            std::vector<moveit_msgs::CollisionObject> collisionObjects;
            collisionObjects.push_back(box);
            ros::Duration(2).sleep();
            virtual_world.addCollisionObjects(collisionObjects);
            ROS_INFO_STREAM("Added: " << blk_name);
        }

        void MyPlanningClass::addObjects()
        {
            double box_pose1[3] = {0.60, -0.67, 0.0,};
            makeBox("block_1", box_pose1);

            double box_pose2[3] = {0.0, 0.77, 0.0,};
            makeBox("block_2", box_pose2);
        }

        void MyPlanningClass::removeObjects()
        {
            std::vector<std::string> object_ids;
            object_ids.push_back("block_1");
            object_ids.push_back("block_2");
            virtual_world.removeCollisionObjects(object_ids);
        }

        void MyPlanningClass::closedGripper(trajectory_msgs::JointTrajectory& posture)
        {
            // BEGIN_SUB_TUTORIAL closed_gripper
            /* Add both finger joints of panda robot. */
            posture.joint_names.resize(2);
            posture.joint_names[0] = "right_gripper_l_finger"; //TODO Change names
            posture.joint_names[1] = "right_gripper_r_finger";

            /* Set them as closed. */
            posture.points.resize(1);
            posture.points[0].positions.resize(2);
            posture.points[0].positions[0] = 0.00;
            posture.points[0].positions[1] = 0.00;
            posture.points[0].time_from_start = ros::Duration(0.5);
        }

        void MyPlanningClass::openGripper(trajectory_msgs::JointTrajectory& posture)
        {   
            std::cout << this->PLANNING_GROUP << std::endl;
            // std::cout << move_group.Options.group_name_ << std::endl;
            // BEGIN_SUB_TUTORIAL open_gripper
            /* Add both finger joints of panda robot. */
            posture.joint_names.resize(2);
            posture.joint_names[0] = "right_gripper_l_finger_tip"; //TODO Change names
            posture.joint_names[1] = "right_gripper_r_finger_tip";

            /* Set them as open, wide enough for the object to fit. */
            posture.points.resize(1);
            posture.points[0].positions.resize(2);
            posture.points[0].positions[0] = 0.04;
            posture.points[0].positions[1] = 0.04;
            posture.points[0].time_from_start = ros::Duration(0.5);
        }

        void MyPlanningClass::Pick(){
            // Create a vector of grasps to be attempted, currently only creating single grasp.
            // This is essentially useful when using a grasp generator to generate and test multiple grasps.
            moveit_msgs::Grasp n1;
            std::vector<moveit_msgs::Grasp> grasps;
            std::cout << "Resizing Grasps" << std::endl;
            grasps.resize(1);
            
            grasps[0].grasp_pose.header.frame_id = "right_l6"; // This link needs to be the base link
            tf2::Quaternion orientation;
            orientation.setRPY(-this->tau / 4, -this->tau / 8, -this->tau / 4);
            grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
            grasps[0].grasp_pose.pose.position.x = this->pick1.position.x;
            grasps[0].grasp_pose.pose.position.y = this->pick1.position.y;
            grasps[0].grasp_pose.pose.position.z = this->pick1.position.z;

            grasps[0].pre_grasp_approach.direction.header.frame_id = "right_l6";
            grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
            grasps[0].pre_grasp_approach.min_distance = 0.095;
            grasps[0].pre_grasp_approach.desired_distance = 0.115;

            grasps[0].post_grasp_retreat.direction.header.frame_id = "right_l6";
            /* Direction is set as positive z axis */
            grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
            grasps[0].post_grasp_retreat.min_distance = 0.1;
            grasps[0].post_grasp_retreat.desired_distance = 0.25;

            this->openGripper(grasps[0].pre_grasp_posture);

            this->closedGripper(grasps[0].grasp_posture);
            std::cout << "Grasps: " << grasps[0] << std::endl;

            this->move_group.setSupportSurfaceName("table1");

            this->move_group.pick("object", grasps);
        }
    
        void MyPlanningClass::Place()
        {
            // BEGIN_SUB_TUTORIAL place
            // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
            // location in verbose mode." This is a known issue. |br|
            // |br|
            // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
            // a single place location.
            std::vector<moveit_msgs::PlaceLocation> place_location;
            place_location.resize(1);

            // Setting place location pose
            // +++++++++++++++++++++++++++
            place_location[0].place_pose.header.frame_id = "right_l0";
            tf2::Quaternion orientation;
            orientation.setRPY(0, 0, this->tau / 4);  // A quarter turn about the z-axis
            place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

            /* For place location, we set the value to the exact location of the center of the object. */
            place_location[0].place_pose.pose.position.x = 0;
            place_location[0].place_pose.pose.position.y = 0.5;
            place_location[0].place_pose.pose.position.z = 0.5;

            // Setting pre-place approach
            // ++++++++++++++++++++++++++
            /* Defined with respect to frame_id */
            place_location[0].pre_place_approach.direction.header.frame_id = "right_l0";
            /* Direction is set as negative z axis */
            place_location[0].pre_place_approach.direction.vector.z = -1.0;
            place_location[0].pre_place_approach.min_distance = 0.095;
            place_location[0].pre_place_approach.desired_distance = 0.115;

            // Setting post-grasp retreat
            // ++++++++++++++++++++++++++
            /* Defined with respect to frame_id */
            place_location[0].post_place_retreat.direction.header.frame_id = "right_l0";
            /* Direction is set as negative y axis */
            place_location[0].post_place_retreat.direction.vector.y = -1.0;
            place_location[0].post_place_retreat.min_distance = 0.1;
            place_location[0].post_place_retreat.desired_distance = 0.25;

            // Setting posture of eef after placing object
            // +++++++++++++++++++++++++++++++++++++++++++
            /* Similar to the pick case */
            this->openGripper(place_location[0].post_place_posture);

            // Set support surface as table2.
            this->move_group.setSupportSurfaceName("table2");
            // Call place to place the object using the place locations given.
            this->move_group.place("object", place_location);
            // END_SUB_TUTORIAL
        }
}