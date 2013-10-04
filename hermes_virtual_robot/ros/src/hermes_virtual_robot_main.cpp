#include "hermes_virtual_robot/HermesVirtualRobot.h"

// Depend packages: urdf
//				    roslib

int main(int argc, char** argv){



	  ros::init(argc, argv, "hermes_virtual_robot");

	  HermesVirtualRobot hermes_virtual;

	  if(!hermes_virtual.getHermesCorrect())
	  {
		  ROS_ERROR("Failed to load Hermes Virtual Robot");
		  return -1;
	  }



	  return 0;


}
