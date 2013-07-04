#include "ros/ros.h"

// services - here you have to include the header file with exactly the same name as your message in the /srv folder (the Message.h is automatically generated from your Message.srv file during compilation)
#include <hermes_grasp_service/HermesGrasp.h>
#include <hermes_grasp_service/graspHand.h>

class HermesGraspServiceServer
{
public:
	HermesGraspServiceServer(ros::NodeHandle nh);
	void init();
	bool executeGrasp(hermes_grasp_service::HermesGrasp::Request &req, hermes_grasp_service::HermesGrasp::Response &res);

protected:
	ros::NodeHandle node_;
	ros::ServiceServer hermes_grasp_service_server_; ///< Service server which accepts requests for squaring a number

	GraspHand left_hand_;
	GraspHand right_hand_;
};
