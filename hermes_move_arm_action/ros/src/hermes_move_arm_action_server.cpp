#include "hermes_move_arm_action/hermes_move_arm_action_server.h"

HermesMoveArmActionServer::HermesMoveArmActionServer(ros::NodeHandle nh)
: move_arm_action_server_(nh, "move_arm_action", boost::bind(&HermesMoveArmActionServer::moveArm, this, _1), false)	// this initializes the action server; important: always set the last parameter to false
{
	node_ = nh;
}


void HermesMoveArmActionServer::init()
{
	// by starting the action server, your action gets advertised to other software modules

	move_arm_action_server_.start();
}


void HermesMoveArmActionServer::moveArm(const hermes_move_arm_action::MoveArmGoalConstPtr& goal)
{
	// this callback function is executed each time a request (= goal message) comes in for this service server
	ROS_INFO("Action Server: Received a request for arm %i.", goal->arm);

	// this command sends a feedback message to the caller, here we transfer that the task is completed 25%
//	hermes_move_arm_action::MoveArmFeedback feedback;
//	feedback.percentageDone = 25;
//	move_arm_action_server_.publishFeedback(feedback);

	// move the arm
	// ...


	hermes_move_arm_action::MoveArmResult res;
	res.return_value.val = arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS; 	// put in there some error code on errors


	// this sends the response back to the caller
	move_arm_action_server_.setSucceeded(res);

	// if failed, use: move_arm_action_server_.setAborted(res);
}
