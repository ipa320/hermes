#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>



int main(int argc, char **argv){

	ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);

	// start a ROS spinning thread
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// this connecs to a running instance of the move_group node
	move_group_interface::MoveGroup group("r_arm");


	// Create geometry for target pose
	geometry_msgs::PoseStamped initPose;
	initPose.header.frame_id = "pillar";
	initPose.pose.position.x = 0.238118 ;
	initPose.pose.position.y = -0.606251;
	initPose.pose.position.z = 0.8506552;
	initPose.pose.orientation.x = 0.632704 ;
	initPose.pose.orientation.y = 0.469899;
	initPose.pose.orientation.z = 0.605991;
	initPose.pose.orientation.w = 0.10796;


	// Create geometry for target pose
	geometry_msgs::PoseStamped goalPose;
	goalPose.header.frame_id = "pillar";
	goalPose.pose.position.x = 0.25457 ;
	goalPose.pose.position.y = -0.836039;
	goalPose.pose.position.z = 1.09907;
	goalPose.pose.orientation.x = 0.70244 ;
	goalPose.pose.orientation.y = 0.411924;
	goalPose.pose.orientation.z = 0.54809;
	goalPose.pose.orientation.w = 0.191031;




	// specify that our target will be a random one
	group.setPoseTarget(goalPose,"r_eef");
	//group.setRandomTarget();

	// plan the motion and then move the group to the sampled target4.0
	group.move();


	// TEST WITH STRAIGHT PATH BUT DOESN'T WORK
	/*std::vector<geometry_msgs::Pose> poses;
	moveit_msgs::RobotTrajectory trajectory;
	move_group_interface::MoveGroup::Plan plan;

	//poses.push_back(initPose.pose);
	poses.push_back(goalPose.pose);
	poses.push_back(initPose.pose);
	group.setPoseReferenceFrame("pillar");


	double fraction = group.computeCartesianPath(poses, 1, 4.0, trajectory);
	std::cout << fraction << std::endl;

	//std::cout << trajectory.joint_trajectory.points.size() << std::endl;
	std::cout << trajectory.joint_trajectory.points[5].velocities[6] << std::endl;

	if(fraction == 1.0) // 100% of path was followed
	{
	    plan.trajectory_ = trajectory;
	    group.execute(plan);
	}*/


	ros::waitForShutdown();

	return 0;
}
