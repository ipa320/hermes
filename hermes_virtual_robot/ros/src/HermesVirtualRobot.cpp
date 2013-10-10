/*
 * HermesVirtualRobot.cpp
 *
 *  Created on: 04/10/2013
 *      Author: ricardo
 */


#include "hermes_virtual_robot/HermesVirtualRobot.h"



// Constructor of the class
HermesVirtualRobot::HermesVirtualRobot(ros::NodeHandle node): node_(node), hermes_correct(true),is_publish(false),
action_server_right_arm_(node, "/right_arm/trajectory_execution_event",boost::bind(&HermesVirtualRobot::executeTrajRightArmCB, this, _1),false),
action_server_left_arm_(node, "/left_arm/trajectory_execution_event",boost::bind(&HermesVirtualRobot::executeTrajLeftArmCB, this, _1),false)
{

	// Find for robot_description param
	if(node_.hasParam("robot_description"))
	{
		std::string robot_name;
		robot_model_loader_ = robot_model_loader::RobotModelLoader("robot_description");
		kinematic_model_ = robot_model_loader_.getModel();
		robot_model_ = kinematic_model_.get();
		robot_name = robot_model_->getName();
		if(robot_name.compare("HermesRobot") !=0)
		{
			hermes_correct = false;
			ROS_ERROR("/robot_description is not a Hermes Robot." );
			return;
		}
		robot_state::RobotStatePtr tmp(new robot_state::RobotState(kinematic_model_));
		kinematic_state_ =   tmp;
		joint_state_group_right_arm_ = kinematic_state_->getJointStateGroup("r_arm");
		joint_state_group_left_arm_ = kinematic_state_->getJointStateGroup("l_arm");

	}
	else
	{
		hermes_correct = false;
		ROS_ERROR("No /robot_description found." );
		return;
	}




	action_server_right_arm_.start();
	action_server_left_arm_.start();

	pub_controller_command_left_ =
		    	      node_.advertise<trajectory_msgs::JointTrajectory>("command", 1);
	pub_controller_command_right_ =
			    	      node_.advertise<trajectory_msgs::JointTrajectory>("command", 1);



	// First Read hermes urdf file
	//readUrdfFile();  // realmente  no hace falta porque viene del param





    joint_states_service_ = node_.advertiseService("hermes_joint_states", &HermesVirtualRobot::getJointStateServer,this);

    grasp_hand_ = node_.advertiseService("grasp_hand", &HermesVirtualRobot::graspCB,this);

	ROS_INFO("Model frame: %s", kinematic_model_->getModelFrame().c_str());


	kinematic_state_->setToDefaultValues();

	// Poner al robot en una posicion aleatoria
	//joint_state_group_right_arm_->setToRandomValues();
	//joint_state_group_left_arm_->setToRandomValues();

	// Poner al robot en la posicion inicial


	kinematic_state_->getStateValues(joint_state_values);
	joint_state_values["r_joint1"] = 0.8339;
	joint_state_values["r_joint2"] = 0.2457;
	joint_state_values["r_joint3"] = -0.3084;
	joint_state_values["r_joint4"] = 0.9955;
	joint_state_values["r_joint5"] = -0.4427;
	joint_state_values["r_joint6"] = 1.1587;
	joint_state_values["r_joint7"] = 2.1482;


	joint_state_values["l_joint1"] = -0.2618;
	joint_state_values["l_joint2"] = 0.3491;
	joint_state_values["l_joint3"] = 2.7925;
	joint_state_values["l_joint4"] = -1.0472;
	joint_state_values["l_joint5"] = -0.0873;
	joint_state_values["l_joint6"] = -0.7854;
	joint_state_values["l_joint7"] = -1.8326;


	// Thumb in correct Position
	joint_state_values["r_thumb_joint1"] = 1.5708;
	joint_state_values["l_thumb_joint1"] = 1.5708;

	kinematic_state_->setStateValues(joint_state_values);







	/*const Eigen::Affine3d &end_effector_state = joint_state_group_right_arm_->getRobotState()->getLinkState("r_eef")->getGlobalLinkTransform();

	std::vector<double> joint_values;
	joint_state_group_right_arm_->getVariableValues(joint_values);

	 for(int i=0;i<joint_values.size();++i)
		 std::cout << "Joint " << i << " Value: " << joint_values[i] << std::endl;*/

	// Vease: http://moveit.ros.org/wiki/Kinematics/C%2B%2B_API
	//*/
}



void HermesVirtualRobot::readUrdfFile()
{
	std::string path = ros::package::getPath("hermes_virtual_robot");
	path += "/ros/urdf/hermes.urdf";

	urdf::Model model;
	if (!model.initFile(path)){
		ROS_ERROR("Failed to parse urdf file");
		hermes_correct = false;

	}
	hermes_model=model;
	// Ajuste del param server para robot_description
	// Lee todo el fichero urdf y pasa como parámetro

	/*std::ifstream file(path.c_str());
	if(file){
		file.seekg(0,file.end);
		int length = file.tellg();
		file.seekg (0, file.beg);
		char * buffer = new char [length];
		file.read (buffer,length);
		node_.setParam("/robot_description", buffer);
		ROS_INFO("Successfully robot_description parsed: %s", path.c_str());

	}*/
	// ES MEJOR CARGAR EN PARAM DESDE EL LAUNCH


	ROS_INFO("Successfully parsed file: %s", path.c_str());


}




// Funciones de movimiento
void HermesVirtualRobot::moveVirtualRightArm()
{


		int nPoints = current_traj_right_.points.size() - 1;


		ros::Duration spendTime;
		ros::Duration trajTime;
		ros::Time beginTime;
		ros::Time endTime;
		ros::Time currentTime;
		ros::Time oldTime;

		beginTime = ros::Time::now();
		currentTime = ros::Time::now();
		trajTime = current_traj_right_.points[nPoints].time_from_start;

		endTime = beginTime+trajTime;

		int cnt = 1;
		oldTime =beginTime;

		std::map< std::string, double > joint_state_values_right;
		while (currentTime<=endTime)
		{


			spendTime = currentTime-oldTime;
			kinematic_state_->getStateValues(joint_state_values_right);
			joint_state_values_right["r_joint1"] += current_traj_right_.points[cnt].velocities[0]*spendTime.toSec();
			joint_state_values_right["r_joint2"] += current_traj_right_.points[cnt].velocities[1]*spendTime.toSec();
			joint_state_values_right["r_joint3"] += current_traj_right_.points[cnt].velocities[2]*spendTime.toSec();
			joint_state_values_right["r_joint4"] += current_traj_right_.points[cnt].velocities[3]*spendTime.toSec();
			joint_state_values_right["r_joint5"] += current_traj_right_.points[cnt].velocities[4]*spendTime.toSec();
			joint_state_values_right["r_joint6"] += current_traj_right_.points[cnt].velocities[5]*spendTime.toSec();
			joint_state_values_right["r_joint7"] += current_traj_right_.points[cnt].velocities[6]*spendTime.toSec();
			joint_state_group_right_arm_->setVariableValues(joint_state_values_right);


			oldTime=currentTime;
			currentTime = ros::Time::now();
			if(cnt+1<nPoints){
				trajTime = current_traj_right_.points[cnt+1].time_from_start;
				if(currentTime>beginTime+trajTime)
					cnt=cnt+1;
			}

		}

		ROS_INFO("TRAJECTORY RIGHT ARM FINISHED");


}

void HermesVirtualRobot::moveVirtualLeftArm()
{


			int nPoints = current_traj_left_.points.size() - 1;


			ros::Duration spendTime;
			ros::Duration trajTime;
			ros::Time beginTime;
			ros::Time endTime;
			ros::Time currentTime;
			ros::Time oldTime;

			beginTime = ros::Time::now();
			currentTime = ros::Time::now();
			trajTime = current_traj_left_.points[nPoints].time_from_start;

			endTime = beginTime+trajTime;

			int cnt = 1;
			oldTime =beginTime;
			std::map< std::string, double > joint_state_values_left;



			while (currentTime<=endTime)
			{

				spendTime = currentTime-oldTime;
				kinematic_state_->getStateValues(joint_state_values_left);
				joint_state_values_left["l_joint1"] += current_traj_left_.points[cnt].velocities[0]*spendTime.toSec();
				joint_state_values_left["l_joint2"] += current_traj_left_.points[cnt].velocities[1]*spendTime.toSec();
				joint_state_values_left["l_joint3"] += current_traj_left_.points[cnt].velocities[2]*spendTime.toSec();
				joint_state_values_left["l_joint4"] += current_traj_left_.points[cnt].velocities[3]*spendTime.toSec();
				joint_state_values_left["l_joint5"] += current_traj_left_.points[cnt].velocities[4]*spendTime.toSec();
				joint_state_values_left["l_joint6"] += current_traj_left_.points[cnt].velocities[5]*spendTime.toSec();
				joint_state_values_left["l_joint7"] += current_traj_left_.points[cnt].velocities[6]*spendTime.toSec();
				joint_state_group_left_arm_->setVariableValues(joint_state_values_left);


				oldTime=currentTime;
				currentTime = ros::Time::now();
				if(cnt+1<nPoints){
					trajTime = current_traj_left_.points[cnt+1].time_from_start;
					if(currentTime>beginTime+trajTime)
						cnt=cnt+1;
				}

			}

			ROS_INFO("TRAJECTORY LEFT ARM FINISHED");


}



bool HermesVirtualRobot::getHermesCorrect()
{
	return hermes_correct;
}



// Execute Moveit Trajectories Callback
void HermesVirtualRobot::executeTrajRightArmCB(GoalHandle gh)
{


	ROS_INFO("NOTE: EXECUTION IN RIGHT ARM VIRTUAL ROBOT. from HermesVirtualRobot.cpp");
	gh.setAccepted();
	active_goal_right_ = gh;

	current_traj_right_ = active_goal_right_.getGoal()->trajectory;
	pub_controller_command_right_.publish(current_traj_right_);
	moveVirtualRightArm();
	JTAS::Result result;
	result.error_code = JTAS::Result::SUCCESSFUL;
	gh.setSucceeded(result);






}
void HermesVirtualRobot::executeTrajLeftArmCB(GoalHandle gh)
{


	ROS_INFO("NOTE: EXECUTION IN LEFT ARM VIRTUAL ROBOT. from HermesVirtualRobot.cpp");
	gh.setAccepted();
	active_goal_left_ = gh;

	current_traj_left_ = active_goal_left_.getGoal()->trajectory;

	pub_controller_command_left_.publish(current_traj_left_);
	moveVirtualLeftArm();
	JTAS::Result result;
	result.error_code = JTAS::Result::SUCCESSFUL;
	gh.setSucceeded(result);



}

bool HermesVirtualRobot::getJointStateServer(hermes_virtual_robot::HermesJointStates::Request  &req, hermes_virtual_robot::HermesJointStates::Response &res)
{

	sensor_msgs::JointState joint_msg;

	kinematic_state_->getStateValues(joint_msg);
	joint_msg.header.stamp=ros::Time::now();

	res.hermes_joint_states = joint_msg;
	return true;

}


bool HermesVirtualRobot::graspCB(hermes_virtual_robot::GraspHand::Request  &req, hermes_virtual_robot::GraspHand::Response &res)
{
	kinematic_state_->getStateValues(joint_state_values);

	double ang=1.5708/2;

	if(req.hand == 2){ //Left Hand
		if(req.grasp_type==1){
			joint_state_values["r_thumb_joint1"] = 1.5708;
			joint_state_values["r_thumb_joint2"] = 0;
			joint_state_values["r_thumb_joint3"] = 0;
			joint_state_values["r_index_joint1"] = 0;
			joint_state_values["r_index_joint2"] = 0;
			joint_state_values["r_middle_joint1"] = 0;
			joint_state_values["r_middle_joint2"] = 0;
			joint_state_values["r_ring_joint1"] = 0;
			joint_state_values["r_ring_joint2"] = 0;
			joint_state_values["r_pinky_joint1"] = 0;
			joint_state_values["r_pinky_joint2"] = 0;
		}
		else if(req.grasp_type==2){
			joint_state_values["r_thumb_joint1"]  = ang;
			joint_state_values["r_thumb_joint2"]  = ang;
			joint_state_values["r_thumb_joint3"]  = ang;
			joint_state_values["r_index_joint1"]  = ang;
			joint_state_values["r_index_joint2"]  = ang;
			joint_state_values["r_middle_joint1"] = ang;
			joint_state_values["r_middle_joint2"] = ang;
			joint_state_values["r_ring_joint1"]   = ang;
			joint_state_values["r_ring_joint2"]   = ang;
			joint_state_values["r_pinky_joint1"]  = ang;
			joint_state_values["r_pinky_joint2"]  = ang;
			}
		else if(req.grasp_type==3){
			joint_state_values["r_thumb_joint1"]  = 0;
			joint_state_values["r_thumb_joint2"]  = 0;
			joint_state_values["r_thumb_joint3"]  = 0;
			joint_state_values["r_index_joint1"]  = 0;
			joint_state_values["r_index_joint2"]  = 0;
			joint_state_values["r_middle_joint1"] = 0;
			joint_state_values["r_middle_joint2"] = 0;
			joint_state_values["r_ring_joint1"]   = 0;
			joint_state_values["r_ring_joint2"]   = 0;
			joint_state_values["r_pinky_joint1"]  = 0;
			joint_state_values["r_pinky_joint2"]  = 0;
		}
		else{
				res.message = "Wrong Type Specified";
				return -1;
		}

	}
	else if (req.hand == 1){
		if(req.grasp_type==1){
			joint_state_values["l_thumb_joint1"] = 1.5708;
			joint_state_values["l_thumb_joint2"] = 0;
			joint_state_values["l_thumb_joint3"] = 0;
			joint_state_values["l_index_joint1"] = 0;
			joint_state_values["l_index_joint2"] = 0;
			joint_state_values["l_middle_joint1"] = 0;
			joint_state_values["l_middle_joint2"] = 0;
			joint_state_values["l_ring_joint1"] = 0;
			joint_state_values["l_ring_joint2"] = 0;
			joint_state_values["l_pinky_joint1"] = 0;
			joint_state_values["l_pinky_joint2"] = 0;
		}
		else if(req.grasp_type==2){
			joint_state_values["l_thumb_joint1"]  = ang;
			joint_state_values["l_thumb_joint2"]  = ang;
			joint_state_values["l_thumb_joint3"]  = ang;
			joint_state_values["l_index_joint1"]  = ang;
			joint_state_values["l_index_joint2"]  = ang;
			joint_state_values["l_middle_joint1"] = ang;
			joint_state_values["l_middle_joint2"] = ang;
			joint_state_values["l_ring_joint1"]   = ang;
			joint_state_values["l_ring_joint2"]   = ang;
			joint_state_values["l_pinky_joint1"]  = ang;
			joint_state_values["l_pinky_joint2"]  = ang;
			}
		else if(req.grasp_type==3){
			joint_state_values["l_thumb_joint1"]  = 0;
			joint_state_values["l_thumb_joint2"]  = 0;
			joint_state_values["l_thumb_joint3"]  = 0;
			joint_state_values["l_index_joint1"]  = 0;
			joint_state_values["l_index_joint2"]  = 0;
			joint_state_values["l_middle_joint1"] = 0;
			joint_state_values["l_middle_joint2"] = 0;
			joint_state_values["l_ring_joint1"]   = 0;
			joint_state_values["l_ring_joint2"]   = 0;
			joint_state_values["l_pinky_joint1"]  = 0;
			joint_state_values["l_pinky_joint2"]  = 0;
		}
		else{
				res.message = "Wrong Type Specified";
				return -1;
		}

	}
	else{
		res.message = "Wrong Hand Specified";
		return -1;
	}




	kinematic_state_->setStateValues(joint_state_values);


	res.message = "OK";
}
