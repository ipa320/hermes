
#ifndef TRAJ_ACTION_SERVER_H_
#define TRAJ_ACTION_SERVER_H_

#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "hermes_move_arm_action/HermesInterface.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "hermes_move_arm_action/HermesInterface.h"


class TrajActionServer
{
private:
	typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JTAS;
	typedef JTAS::GoalHandle GoalHandle;
	ros::NodeHandle node_;
	JTAS action_server_;
	GoalHandle active_goal_;
	ros::Publisher pub_controller_command_;
	 trajectory_msgs::JointTrajectory current_traj_;

public:
	TrajActionServer(ros::NodeHandle &n);
	~TrajActionServer();
	void getJointStateRightArm(std::vector<float> &q);

protected:
	HermesInterface hermesinterface;

private:
	void executeTrajCB(GoalHandle gh);
	void cancelCB(GoalHandle gh);





};



#endif /* TRAJ_ACTION_SERVER_H_ */
