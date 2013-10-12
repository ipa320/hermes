#include <moveit/move_group_interface/move_group.h>
#include "ros/ros.h"

// From : http://moveit.ros.org/wiki/MoveGroup_Interface#motion_plan


int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
	// start a ROS spinning thread
	ros::AsyncSpinner spinner(1);
	spinner.start();
	// this connecs to a running instance of the move_group node
	move_group_interface::MoveGroup group_right("r_arm");
	move_group_interface::MoveGroup group_left("l_arm");
	// specify that our target will be a random one
	group_right.setRandomTarget();
	group_left.setRandomTarget();
	bool have_plan_right = false;
	bool have_plan_left = false;
	moveit::planning_interface::MoveGroup::Plan plan_right;
	moveit::planning_interface::MoveGroup::Plan plan_left;

	for (int trial=0; have_plan_right==false && trial<5; ++trial)
		have_plan_right = group_right.plan(plan_right);
	if (have_plan_right==true)
		group_right.execute(plan_right);
	else
		ROS_WARN("No valid plan for right found for right arm movement.");

	for (int trial=0; have_plan_left==false && trial<5; ++trial)
		have_plan_left = group_left.plan(plan_left);
	if (have_plan_left==true)
		group_left.execute(plan_left);
	else
		ROS_WARN("No valid plan for left found for left arm movement.");

  // plan the motion and then move the group to the sampled target

  ros::waitForShutdown();
}

