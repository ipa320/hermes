#include "ros/ros.h"


#include <hermes_reference_frames_service/HermesFrame.h>
#include <iostream>

int main(int argc, char **argv)
{
	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line. For programmatic
	* remappings you can use a different version of init() which takes remappings
	* directly, but for most command-line programs, passing argc and argv is the easiest
	* way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "reference_frames_service_client");


	ros::NodeHandle n;

	std::string service_name = "/reference_frames_service";

	// here we wait until the service is available; please use the same service name as the one in the server; you may define a timeout if the service does not show up
	std::cout << "Waiting for service server to become available..." << std::endl;
	bool serviceAvailable = ros::service::waitForService(service_name, 5000);

	// only proceed if the service is available
	if (serviceAvailable == false)
	{
		std::cout << "The service could not be found.\n" << std::endl;
		return -1;
	}
	std::cout << "The service server is advertised.\n" << std::endl;

	// prepare the request and response messages
	hermes_reference_frames_service::HermesFrame::Request req;
	hermes_reference_frames_service::HermesFrame::Response res;

	while(1)
	{
		std::cout << "Enter Reference frame  (1-2): ";
		std::cin >> req.frame; // = hermes_grasp_service::HermesGrasp::Request::LEFTHAND;
		if (req.frame == 0)
			break;


		// this calls the service server to process our request message and put the result into the response message
		// this call is blocking, i.e. this program will not proceed until the service server sends the response
		bool success = ros::service::call(service_name, req, res);

		if (success == true){
			std::cout << "Pos X:" << res.position[0] << std::endl;
			std::cout << "Pos Y:" << res.position[1] << std::endl;
			std::cout << "Pos Z:" << res.position[2] << std::endl;

			std::cout << "Rot W:" << res.quaternion[0] << std::endl;
			std::cout << "Rot X:" << res.quaternion[1] << std::endl;
			std::cout << "Rot Y:" << res.quaternion[2] << std::endl;
			std::cout << "Rot Z:" << res.quaternion[3] << std::endl;
		}
		else
			std::cout << "The service call was not successful.\n" << std::endl;
	}

	return 0;
}
