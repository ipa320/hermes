#include "ros/ros.h"

#include "hermes_arm_kdl/hermes_arm_kdl_service_server.h"


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "arm_kdl_service_server");

	
	ros::NodeHandle n;

	// create an instance of the exemplary service server class
	HermesArmKdlServiceServer srv(n);

	// initialize the service server (for details see comments in class)
	srv.init();

	/**
	* ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	*/
	ros::spin();

	return 0;
}
