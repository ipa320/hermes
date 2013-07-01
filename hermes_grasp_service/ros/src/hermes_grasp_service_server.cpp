#include "hermes_grasp_service/hermes_grasp_service_server.h"

HermesGraspServiceServer::HermesGraspServiceServer(ros::NodeHandle nh)
{
	node_ = nh;
}


void HermesGraspServiceServer::init()
{
	/**
	* The advertiseService() function is how you tell ROS that you want to provide a service for other modules (software nodes).
	*/

	hermes_grasp_service_server_ = node_.advertiseService("hermes_grasp_service", &HermesGraspServiceServer::executeGrasp, this);

	left_hand_.init(1);
	right_hand_.init(2);
}


bool HermesGraspServiceServer::executeGrasp(hermes_grasp_service::Grasp::Request &req, hermes_grasp_service::Grasp::Response &res)
{
	// this callback function is executed each time a request comes in for this service server
	// here we just read the number from the request, square it and put the result into the response, the response is automatically sent back to the caller when this function returns

	ROS_INFO("Service Server: Received a request with hand %i grasp type %i and grasp force %i.",req.hand, req.grasp_type, req.grasp_force);

	// code for grasp execution
	if (req.hand == hermes_grasp_service::Grasp::Request::LEFTHAND)
		left_hand_.executeGrasp(req.grasp_type, req.grasp_force);
	else if (req.hand == hermes_grasp_service::Grasp::Request::RIGHTHAND)
		right_hand_.executeGrasp(req.grasp_type, req.grasp_force);
	else
	{
		res.message = "wrong hand specified.";
		return false;
	}
	
	// if the procedure fails, use "return false;" to inform the caller of the service
	res.message = "success";
	return true;	// tell the caller that the method was successfully executed
}
