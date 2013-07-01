#include "hermes_grasp_database/hermes_grasp_database.h"

#include "geometry_msgs/PoseStamped.h"


HermesGraspDatabase::HermesGraspDatabase(ros::NodeHandle nh)
{
	node_ = nh;
}


void HermesGraspDatabase::init(std::string databaseFile)
{
	loadDatabase(databaseFile);

	/**
	* The advertiseService() function is how you tell ROS that you want to provide a service for other modules (software nodes).
	*/
	hermes_grasp_database_service_server_ = node_.advertiseService("hermes_get_grasp_for_detection", &HermesGraspDatabase::computeGraspForDetection, this);
}


void HermesGraspDatabase::loadDatabase(std::string databaseFile)
{

}


bool HermesGraspDatabase::computeGraspForDetection(hermes_grasp_database::GetGraspForDetection::Request &req, hermes_grasp_database::GetGraspForDetection::Response &res)
{
	// this callback function is executed each time a request comes in for this service server
	// here we just read the number from the request, square it and put the result into the response, the response is automatically sent back to the caller when this function returns

	ROS_INFO("Service Server: Received a request for detected object %s at (%f, %f, %f).", req.detection.label.c_str(), req.detection.pose.pose.position.x, req.detection.pose.pose.position.y, req.detection.pose.pose.position.z);

	// code for grasp database lookup
	// ...
	//tf::Transform t;

	// if the procedure fails, use "return false;" to inform the caller of the service
	res.goal_position = geometry_msgs::PoseStamped();
	return true;	// tell the caller that the method was successfully executed
}
