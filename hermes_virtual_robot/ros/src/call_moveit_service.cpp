#include <moveit/move_group_interface/move_group.h>
#include "ros/ros.h"

// From : http://moveit.ros.org/wiki/MoveGroup_Interface#motion_plan


int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
	// start a ROS spinning thread
	ros::AsyncSpinner spinner(2);
	spinner.start();
	// this connecs to a running instance of the move_group node
	move_group_interface::MoveGroup group_right("r_arm");
	move_group_interface::MoveGroup group_left("l_arm");

	// specify that our target will be a random one
	group_right.setRandomTarget();
	group_left.setRandomTarget();


	// Para  definir la posición obetivo en cartesianas desde world
	// tengo que calcular de cinematica inversa de movit
	// para obtener las articulares y definir el objetivo

	std::map< std::string, double > joint_state_values;


	group_right.setJointValueTarget(joint_state_values);





	bool have_plan_right = false;
	bool have_plan_left = false;





	moveit::planning_interface::MoveGroup::Plan plan_right;
	moveit::planning_interface::MoveGroup::Plan plan_left;



	for (int trial=0; have_plan_right==false && trial<5; ++trial)
		have_plan_right = group_right.plan(plan_right);



	for (int trial=0; have_plan_left==false && trial<5; ++trial)
			have_plan_left = group_left.plan(plan_left);


	/* TRAJECTORY EXECUTION
	if ( have_plan_left==true){
		group_left.asyncExecute(plan_left);
	}
	else
			ROS_WARN("No valid plan for right found for left arm movement.");
	if ( have_plan_right==true){
			group_right.asyncExecute(plan_right);

		}
	else
		ROS_WARN("No valid plan for right found for left arm movement.");*/


  ros::waitForShutdown();
}




