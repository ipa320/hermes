#include "hermes_virtual_robot/HermesVirtualRobot.h"

// Depend packages: urdf
//				    roslib

int main(int argc, char** argv){



	  ros::init(argc, argv, "hermes_virtual_robot");
	  ros::NodeHandle n;
	  ros::MultiThreadedSpinner spinner(4);

	  ros::Rate loop_rate(30);


	  HermesVirtualRobot hermes_virtual(n);

	  if(!hermes_virtual.getHermesCorrect())
	  {
		  ROS_ERROR("Failed to load Hermes Virtual Robot");
		  return -1;
	  }
	  spinner.spin();
	 /* while (ros::ok()) {
		  std::cout << "publicando" << std::endl;
		  hermes_virtual.publish_robot_state();
		  ros::spinOnce();
		  loop_rate.sleep();
	  }*/

	  return 0;



}
