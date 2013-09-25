#include "hermes_traj_action/traj_action_server.h"



TrajActionServer::TrajActionServer(ros::NodeHandle &n) : node_(n), action_server_(node_, "traj_action",
                       boost::bind(&TrajActionServer::executeTrajCB, this, _1),
                       boost::bind(&TrajActionServer::cancelCB, this, _1),
                       false)
{

	pub_controller_command_ =
    	      node_.advertise<trajectory_msgs::JointTrajectory>("command", 1);

	action_server_.start();


}
TrajActionServer::~TrajActionServer()
{
       pub_controller_command_.shutdown();
       /*sub_controller_state_.shutdown();
        watchdog_timer_.stop();*/
}

void TrajActionServer::executeTrajCB(GoalHandle gh)
{
	std::cout << "EJECUTANDO executeTrajCB	holaaaaa" << std::endl;
	gh.setAccepted();
			active_goal_ = gh;
	current_traj_ = active_goal_.getGoal()->trajectory;
	pub_controller_command_.publish(current_traj_);

	int nPoints = current_traj_.points.size() - 1;

	for(int i=0;i<=nPoints;i++){

		std::cout << current_traj_.points[i].velocities[0]  << std::endl;
		ros::Duration(current_traj_.points[i].time_from_start).sleep();
	}
}

void TrajActionServer::cancelCB(GoalHandle gh)
{
	std::cout << "TRAYECTORY FINISHED	" << std::endl;
}

