/*
 * HermesVirtualRobot.cpp
 *
 *  Created on: 04/10/2013
 *      Author: ricardo
 */


#include "hermes_virtual_robot/HermesVirtualRobot.h"


// Constructor of the class
HermesVirtualRobot::HermesVirtualRobot(): hermes_correct(true)
{
	// First Read hermes urdf file
	readUrdfFile();
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
	ROS_INFO("Successfully parsed file: %s", path.c_str());


}

bool HermesVirtualRobot::getHermesCorrect()
{
	return hermes_correct;
}


