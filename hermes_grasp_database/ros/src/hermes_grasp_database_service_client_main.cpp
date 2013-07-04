#include "ros/ros.h"

// services - here you have to include the header file with exactly the same name as your message in the /srv folder (the Message.h is automatically generated from your Message.srv file during compilation)
#include <hermes_grasp_database/GetGraspForDetection.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
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
	ros::init(argc, argv, "grasp_database_client");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;

	std::string service_name = "/hermes_grasp_database/get_grasp_for_detection";

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
	hermes_grasp_database::GetGraspForDetection::Request req;
	hermes_grasp_database::GetGraspForDetection::Response res;

	req.detection.label = "test_object";
	req.detection.pose.pose.position.x = 0.834;
	req.detection.pose.pose.position.y = 0.328;
	req.detection.pose.pose.position.z = 1.135;
	req.detection.pose.pose.orientation.w = 1;
	req.detection.pose.pose.orientation.x = 0;
	req.detection.pose.pose.orientation.y = 0;
	req.detection.pose.pose.orientation.z = 0;

	// this calls the service server to process our request message and put the result into the response message
	// this call is blocking, i.e. this program will not proceed until the service server sends the response
	bool success = ros::service::call(service_name, req, res);

	if (success == true)
	{
		printf("Request successful, target arm positions:\n");
		for (unsigned int i=0; i<res.grasp_configurations.size(); i++)
			printf(" - (xyz)=(%f, %f, %f), (wabc)=(%f, %f, %f, %f) with grasp %i and force %i\n",
				res.grasp_configurations[i].goal_position.pose.position.x, res.grasp_configurations[i].goal_position.pose.position.y, res.grasp_configurations[i].goal_position.pose.position.z,
				res.grasp_configurations[i].goal_position.pose.orientation.w, res.grasp_configurations[i].goal_position.pose.orientation.x, res.grasp_configurations[i].goal_position.pose.orientation.y, res.grasp_configurations[i].goal_position.pose.orientation.z,
				res.grasp_configurations[i].grasp_type, res.grasp_configurations[i].grasp_force);
	}
	else
		std::cout << "The service call was not successful.\n" << std::endl;


	return 0;
}
