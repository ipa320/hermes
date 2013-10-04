#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <hermes_robot_interface/MoveArmAction.h>
#include "tf/tf.h"



typedef actionlib::SimpleActionClient<hermes_robot_interface::MoveArmAction> HermesMoveArmClientType;
int main(int argc, char **argv)
{
	ros::init(argc,argv, "hermes_action_client");

	ros::NodeHandle n;

	HermesMoveArmClientType move_arm_action_client_("/move_arm_action", true);

	bool serverAvariable;

	std::cout << "Waiting for action server to become available..." << std::endl;
	serverAvariable = move_arm_action_client_.waitForServer(ros::Duration(5.0));

	if(serverAvariable)
		std::cout << "move_arm_action disponible" << std::endl;
	else{
		std::cout << "ERROR: move_arm_action no disponible" << std::endl;
		return -1;
	}

	// prepare the goal message
	hermes_robot_interface::MoveArmGoal goal;
	goal.arm = hermes_robot_interface::MoveArmGoal::RIGHTARM;

	goal.goal_position = geometry_msgs::PoseStamped();
	tf::Transform goalPose(tf::Matrix3x3(-0.7881, -0.2110, 0.5782, 0.5811, 0.0544, 0.8120, -0.2028, 0.9760, 0.0798), tf::Vector3(0.8193, -0.2148, 1.6487));

	goal.goal_position.header.frame_id = "/pillar";
	goal.goal_position.pose.position.x = goalPose.getOrigin().getX();
	goal.goal_position.pose.position.y = goalPose.getOrigin().getY();
	goal.goal_position.pose.position.z = goalPose.getOrigin().getZ();
	goal.goal_position.pose.orientation.x = goalPose.getRotation().getX();
	goal.goal_position.pose.orientation.y = goalPose.getRotation().getY();
	goal.goal_position.pose.orientation.z = goalPose.getRotation().getZ();
	goal.goal_position.pose.orientation.w = goalPose.getRotation().getW();

	move_arm_action_client_.sendGoal(goal);
	//actionlib::SimpleActionClient<learning_actionlib::FibonacciAction> ac("fibonacci", true);


	return 0;
}



