#include "hermes_arm_kdl/Fkine.h"
#include "hermes_arm_kdl/hermes_arm_kdl_service_server.h"


HermesArmKdlServiceServer::HermesArmKdlServiceServer(ros::NodeHandle nh)
{
	node_ = nh;
}


void HermesArmKdlServiceServer::init()
{
	/**
	* The advertiseService() function is how you tell ROS that you want to provide a service for other modules (software nodes).
	*/

	hermes_arm_kdl_service_server_ = node_.advertiseService("arm_kdl_service_server", &HermesArmKdlServiceServer::fkine, this);

}


bool HermesArmKdlServiceServer::fkine(hermes_arm_kdl::Fkine::Request &req, hermes_arm_kdl::Fkine::Response &res)
{
	// this callback function is executed each time a request comes in for this service server
	// here we just read the number from the request, square it and put the result into the response, the response is automatically sent back to the caller when this function returns

	/*ROS_INFO("HermesGrasp Service Server: Received a request with hand %i grasp type %i and grasp force %i.",req.hand, req.grasp_type, req.grasp_force);

	// code for grasp execution
	if (req.hand == hermes_grasp_service::HermesGrasp::Request::LEFTHAND)
		left_hand_.executeGrasp(req.grasp_type, req.grasp_force);
	else if (req.hand == hermes_grasp_service::HermesGrasp::Request::RIGHTHAND)
		right_hand_.executeGrasp(req.grasp_type, req.grasp_force);
	else
	{
		res.message = "wrong hand specified.";
		return false;
	}*/
	
	// if the procedure fails, use "return false;" to inform the caller of the service
	res.message = "success";
	res.fkine = req.test+1;
	return true;	// tell the caller that the method was successfully executed
}
