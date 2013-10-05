/*
 * HermesVirtualRobot.cpp
 *
 *  Created on: 04/10/2013
 *      Author: ricardo
 */


#include "hermes_virtual_robot/HermesVirtualRobot.h"
#include <urdf/model.h>
#include <urdf/link.h>
#include <stdio.h>
#include <fstream>


// Constructor of the class
HermesVirtualRobot::HermesVirtualRobot(ros::NodeHandle node):node_(node), hermes_correct(true)
{
	// First Read hermes urdf file
	readUrdfFile();



	// joint_states advertise
	joint_pub=node_.advertise<sensor_msgs::JointState>("joint_states",1);
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
	// Ajuste del param server para robot descriptio
//	node_.setParam("/robot_description", fopen(path,"r").read());
	std::ifstream file(path.c_str());
	if(file){
		file.seekg(0,file.end);
		int length = file.tellg();
		file.seekg (0, file.beg);
		char * buffer = new char [length];
		file.read (buffer,length);
		node_.setParam("/robot_description", buffer);

	}









	//std::map<std::string,boost::shared_ptr<urdf::Link> > links;
	//links = hermes_model.links_;

	//std::map<std::string,boost::shared_ptr<urdf::Link> >::iterator iter;
	//for(std::map<std::string,boost::shared_ptr<urdf::Link> >::iterator iter=links.begin(); iter!=links.end(); ++iter)
	//{
	//	std::cout << iter->first << " => " << iter->second << std::endl;

	//}

	ROS_INFO("Successfully parsed file: %s", path.c_str());


}

bool HermesVirtualRobot::getHermesCorrect()
{
	return hermes_correct;
}

void HermesVirtualRobot::publish_robot_state()
{



}
