#include "hermes_move_arm_action/hermes_move_arm_action_client.h"

HermesMoveArmActionClient::HermesMoveArmActionClient(ros::NodeHandle nh)
: move_arm_action_client_("/hermes_move_arm_action/move_arm_action", true)  // true -> don't need ros::spin()
{
	node_ = nh;
}


bool HermesMoveArmActionClient::init()
{
	// here we wait until the service is available; please use the same service name as the one in the server; you may define a timeout if the service does not show up
	std::cout << "Waiting for action server to become available..." << std::endl;
	return move_arm_action_client_.waitForServer(ros::Duration(5.0));
}


void HermesMoveArmActionClient::run()
{
	// prepare the goal message
	hermes_move_arm_action::MoveArmGoal goal;

//	while(1)
//	{
		goal.arm = 2;
		goal.goal_position = geometry_msgs::PoseStamped();

		// this calls the action server to process our goal message and send result message which will cause the execution of the doneCb callback function
		// this call is not blocking, i.e. this program can proceed immediately after the action call
		move_arm_action_client_.sendGoal(goal, boost::bind(&HermesMoveArmActionClient::doneCb, this, _1, _2), boost::bind(&HermesMoveArmActionClient::activeCb, this), boost::bind(&HermesMoveArmActionClient::feedbackCb, this, _1));
//	}
}

// Called once when the goal completes
void HermesMoveArmActionClient::doneCb(const actionlib::SimpleClientGoalState& state, const hermes_move_arm_action::MoveArmResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Result: %i", result->return_value.val);
  std::cout << "Computation finished." << std::endl;
}

// Called once when the goal becomes active
void HermesMoveArmActionClient::activeCb()
{
  ROS_INFO("Starting movement of arm.");
}

// Called every time feedback is received for the goal
void HermesMoveArmActionClient::feedbackCb(const hermes_move_arm_action::MoveArmFeedbackConstPtr& feedback)
{
  //ROS_INFO("Computation accomplished by %f percent.", feedback->percentageDone);
}
