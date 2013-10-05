/*
 * HermesVirtualRobot.cpp
 *
 *  Created on: 04/10/2013
 *      Author: ricardo
 */


#include "hermes_virtual_robot/HermesVirtualRobot.h"



// Constructor of the class
HermesVirtualRobot::HermesVirtualRobot(ros::NodeHandle node):node_(node), hermes_correct(true),
action_server_(node, "/trajectory_execution_event",boost::bind(&HermesVirtualRobot::executeTrajCB, this, _1),false)
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






	action_server_.start();
	// First Read hermes urdf file
	//readUrdfFile();  // realmente  no hace falta porque viene del param

	// joint_states advertise
	joint_pub=node_.advertise<sensor_msgs::JointState>("joint_states",1);

	ROS_INFO("Model frame: %s", kinematic_model_->getModelFrame().c_str());


	kinematic_state_->setToDefaultValues();

	// Poner al robot en una posicion aleatoria
	//joint_state_group_right_arm_->setToRandomValues();
	//joint_state_group_left_arm_->setToRandomValues();

	// Poner al robot en la posicion inicial


	kinematic_state_->getStateValues(joint_state_values);
	joint_state_values["r_joint1"] = 0.7291;
	joint_state_values["r_joint2"] = -0.0979;
	joint_state_values["r_joint3"] = -0.9321;
	joint_state_values["r_joint4"] = 1.2126;
	joint_state_values["r_joint5"] = 0.5357;
	joint_state_values["r_joint6"] = 0.1822;
	joint_state_values["r_joint7"] = 2.6213;

	joint_state_values["l_joint1"] = 0.1935;
	joint_state_values["l_joint2"] = 1.0232;
	joint_state_values["l_joint3"] = 2.7270;
	joint_state_values["l_joint4"] = -1.3565;
	joint_state_values["l_joint5"] = 0.4324;
	joint_state_values["l_joint6"] = -0.1122;
	joint_state_values["l_joint7"] = -3.0668;

	joint_state_group_right_arm_->setVariableValues(joint_state_values);
	joint_state_group_left_arm_->setVariableValues(joint_state_values);








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

bool HermesVirtualRobot::getHermesCorrect()
{
	return hermes_correct;
}

void HermesVirtualRobot::publish_robot_state()
{


	//joint_state_group_right_arm_->setToRandomValues();
	kinematic_state_->getStateValues(joint_msg_);
	joint_msg_.header.stamp=ros::Time::now();
	joint_pub.publish(joint_msg_);

}

// Execute Moveit Trajectories Callback
void HermesVirtualRobot::executeTrajCB(GoalHandle gh)
{
	std::cout << "EJECUTANDO TRAYECTORIA EN EL ROBOT VIRTUAL" << std::endl;

}
