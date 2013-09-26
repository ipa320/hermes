#include "ros/ros.h"
#include "std_msgs/String.h"
#include <kdl/frames.hpp>
#include "hermes_virtual_traj/hermes_virtual_robot.h"

int main(int argc,char* argv[]) {

	ros::init(argc, argv, "hermes_virtual_robot");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);

	HermesVirtualRobot hermes(nh);

	while(ros::ok()){
		hermes.publishJointState();
		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}


