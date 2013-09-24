#include "hermes_robot_interface/hermes_robot_interface.h"


HermesRobotInterface::HermesRobotInterface(ros::NodeHandle &nh): node_(nh), action_server_(nh, "traj_action",
       boost::bind(&HermesRobotInterface::executeTrajCB, this, _1),false),
       move_arm_action_server_(nh, "move_arm_action", boost::bind(&HermesRobotInterface::moveArmCB, this, _1),false)
{

	pub_controller_command_ =
	    	      node_.advertise<trajectory_msgs::JointTrajectory>("command", 1);

	action_server_.start();
	move_arm_action_server_.start();
}

void HermesRobotInterface::init()
{

	// todo:  bring-up robot modules
	hermesinterface.init();

	//todo:  JointState inicialize
	jointState.name.resize(36);
	jointState.position.resize(36);
	jointState.velocity.resize(36);
	jointState.effort.resize(36);


	//todo: Put name to JointState
	jointState.name[0] ="r_joint1";
	jointState.name[1] ="r_joint2";
	jointState.name[2] ="r_joint3";
	jointState.name[3] ="r_joint4";
	jointState.name[4] ="r_joint5";
	jointState.name[5] ="r_joint6";
	jointState.name[6] ="r_joint7";
	jointState.name[7] ="l_joint1";
	jointState.name[8] ="l_joint2";
	jointState.name[9] ="l_joint3";
	jointState.name[10] ="l_joint4";
	jointState.name[11] ="l_joint5";
	jointState.name[12] ="l_joint6";
	jointState.name[13] ="l_joint7";
	jointState.name[14] ="r_thumb_joint1";
	jointState.name[15] ="r_thumb_joint2";
	jointState.name[16] ="r_thumb_joint3";
	jointState.name[17] ="r_index_joint1";
	jointState.name[18] ="r_index_joint2";
	jointState.name[19] ="r_middle_joint1";
	jointState.name[20] ="r_middle_joint2";
	jointState.name[21] ="r_ring_joint1";
	jointState.name[22] ="r_ring_joint2";
	jointState.name[23] ="r_pinky_joint1";
	jointState.name[24] ="r_pinky_joint2";
	jointState.name[25] ="l_thumb_joint1";
	jointState.name[26] ="l_thumb_joint2";
	jointState.name[27] ="l_thumb_joint3";
	jointState.name[28] ="l_index_joint1";
	jointState.name[29] ="l_index_joint2";
	jointState.name[30] ="l_middle_joint1";
	jointState.name[31] ="l_middle_joint2";
	jointState.name[32] ="l_ring_joint1";
	jointState.name[33] ="l_ring_joint2";
	jointState.name[34] ="l_pinky_joint1";
	jointState.name[35] ="l_pinky_joint2";
}




void HermesRobotInterface::getJointState(sensor_msgs::JointState &jointState)
{
	std::vector<float> q_left;
	std::vector<float> q_right;

	q_left.resize(7);
	q_right.resize(7);

	hermesinterface.get_leftJoints(q_left);
	hermesinterface.get_rightJoints(q_right);

	for(int i=0;i<36;i++){
			jointState.name[i] = this->jointState.name[i];

	}

	for(int i=0;i<7;i++){
		jointState.position[i] = q_right[i];
		jointState.position[7+i] = q_left[i];
	}


}


void HermesRobotInterface::executeTrajCB(GoalHandle gh)
{
	ROS_INFO("CAUTION: EXECUTION IN REAL ROBOT");
	gh.setAccepted();
			active_goal_ = gh;



	current_traj_ = active_goal_.getGoal()->trajectory;
	pub_controller_command_.publish(current_traj_);

	int nPoints = current_traj_.points.size() - 1;

	std::vector <float> dq_right;
	dq_right.resize(7);

	for(int i=0;i<=nPoints;i++){
		for(int j=0;j<7;j++){
			dq_right[j] = current_traj_.points[i].velocities[j];
		}
		hermesinterface.moveRightArmVel(dq_right);
		//std::cout << current_traj_.points[i].positions[6] << std::endl;
		ros::spinOnce();
		if(i>1)
			ros::Duration(current_traj_.points[i].time_from_start-current_traj_.points[i-1].time_from_start).sleep();
		else
			ros::Duration(current_traj_.points[i].time_from_start).sleep();
		
	}
	JTAS::Result result;
	result.error_code = JTAS::Result::SUCCESSFUL;
	gh.setSucceeded(result);

	hermesinterface.softStopAll();

}

void HermesRobotInterface::moveArmCB(const hermes_robot_interface::MoveArmGoalConstPtr& goal)
{
	// this callback function is executed each time a request (= goal message) comes in for this service server
	ROS_INFO("MoveArm Action Server: Received a request for arm %i.", goal->arm);

//	if (goal->goal_position.header.frame_id.compare("/world") != 0)
//	{
//		ROS_ERROR("The goal position coordinates are not provided in the correct frame. The required frame is '/world' but '%s' was provided.", goal->goal_position.header.frame_id.c_str());
//		return;
//	}

	if(goal->arm == hermes_robot_interface::MoveArmGoal::RIGHTARM)
	{
		// this connecs to a running instance of the move_group node
		move_group_interface::MoveGroup group("r_arm");
		// specify that our target will be a random one
		group.setPoseTarget(goal->goal_position,"r_eef");

		// plan the motion and then move the group to the sampled target4.0
		group.move();
	}

	hermes_robot_interface::MoveArmResult res;
	res.return_value.val = arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS; 	// put in there some error code on errors


	// this sends the response back to the caller
	move_arm_action_server_.setSucceeded(res);


}


