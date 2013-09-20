#include "hermes_move_arm_action/traj_action_server.h"



TrajActionServer::TrajActionServer(ros::NodeHandle &n) : node_(n), action_server_(node_, "traj_action",
                       boost::bind(&TrajActionServer::executeTrajCB, this, _1),
                       boost::bind(&TrajActionServer::cancelCB, this, _1),
                       false)
{

	pub_controller_command_ =
    	      node_.advertise<trajectory_msgs::JointTrajectory>("command", 1);


	hermesinterface.init();
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
	std::cout << "EJECUTANDO executeTrajCB	" << std::endl;

	std::vector <float> qVel;
	qVel.resize(7,0.0);


	gh.setAccepted();
			active_goal_ = gh;
	current_traj_ = active_goal_.getGoal()->trajectory;
	pub_controller_command_.publish(current_traj_);

	int nPoints = current_traj_.points.size() - 1;

	for(int i=0;i<=nPoints;i++){
		for(int j=0;j<7;j++){
			qVel[j] = current_traj_.points[i].velocities[j]/10;
		}
		//hermesinterface.moveRightArmVel(qVel);
		/*std::cout << current_traj_.points[i].velocities[0] << "	" << current_traj_.points[i].velocities[1] << "	" <<
				     current_traj_.points[i].velocities[2] << "	" << current_traj_.points[i].velocities[3] << "	" <<
				     current_traj_.points[i].velocities[4] << "	" << current_traj_.points[i].velocities[5] << "	" <<
				     current_traj_.points[i].velocities[6]  << std::endl;*/
		std::cout << "Time: " << current_traj_.points[i].time_from_start << std::endl;
		if(i>1){
			ros::Duration((current_traj_.points[i].time_from_start-current_traj_.points[i-i].time_from_start)).sleep();
		}
		else
		{
			ros::Duration((current_traj_.points[i].time_from_start)).sleep();
		}
	}
}

void TrajActionServer::cancelCB(GoalHandle gh)
{
	std::cout << "TRAYECTORY FINISHED	" << std::endl;
}

void TrajActionServer::getJointStateRightArm(std::vector<float> &q)
{
	hermesinterface.get_rightJoints(q);
}

