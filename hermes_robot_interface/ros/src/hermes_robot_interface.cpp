#include "hermes_robot_interface/hermes_robot_interface.h"


HermesRobotInterface::HermesRobotInterface(ros::NodeHandle &nh): node_(nh),
	   action_server_(nh, "traj_action_",boost::bind(&HermesRobotInterface::executeTrajCB, this, _1),false),
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

	std::vector <float> dq;
	dq.resize(7);
	ros::Duration spendTime;
	ros::Time beginTime;
	ros::Time endTime;

	for(int i=0;i<=nPoints;i++){
		beginTime = ros::Time::now();
		for(int j=0;j<7;j++){
			if(i==0)
				dq[j] = current_traj_.points[i+1].velocities[j];
			else if(i==nPoints)
				dq[j] = current_traj_.points[i-1].velocities[j];
			else
				dq[j] = current_traj_.points[i].velocities[j];

		}
		// Case RIGHT ARM
		if (current_traj_.joint_names[0].compare("r_joint1")==0)
		{
			hermesinterface.moveRightArmVel(dq);
		}
		if (current_traj_.joint_names[0].compare("l_joint1")==0)
		{
			hermesinterface.moveLeftArmVel(dq);
		}

		endTime = ros::Time::now();
		if(i>=1){
			 spendTime = (current_traj_.points[i].time_from_start - current_traj_.points[i-1].time_from_start)-(endTime-beginTime);
		}
		else{
			 spendTime = (current_traj_.points[i].time_from_start);
		}
		if(spendTime.toSec()>0)
			spendTime.sleep();
		

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

		// plan the motion and then move the group to the sampled target
		bool have_plan = false;
		moveit::planning_interface::MoveGroup::Plan plan;
		for (int trial=0; have_plan==false && trial<5; ++trial)
			have_plan = group.plan(plan);
		if (have_plan==true)
			group.execute(plan);
		else
			ROS_WARN("No valid plan found for arm movement.");
	}
	else if(goal->arm == hermes_robot_interface::MoveArmGoal::LEFTARM){
		// this connecs to a running instance of the move_group node
		move_group_interface::MoveGroup group("l_arm");
		// specify that our target will be a random one
		group.setPoseTarget(goal->goal_position,"l_eef");

		// plan the motion and then move the group to the sampled target
		bool have_plan = false;
		moveit::planning_interface::MoveGroup::Plan plan;
		for (int trial=0; have_plan==false && trial<5; ++trial)
			have_plan = group.plan(plan);
		if (have_plan==true)
			group.execute(plan);
		else
			ROS_WARN("No valid plan found for arm movement.");
	}



	hermes_robot_interface::MoveArmResult res;
	res.return_value.val = arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS; 	// put in there some error code on errors


	// this sends the response back to the caller
	move_arm_action_server_.setSucceeded(res);


}


