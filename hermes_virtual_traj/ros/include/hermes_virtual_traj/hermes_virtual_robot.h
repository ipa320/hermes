/*
 * hermes_virtual.h
 *
 *  Created on: 26/09/2013
 *      Author: vr2user
 */

#ifndef HERMES_VIRTUAL_H_
#define HERMES_VIRTUAL_H_

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <actionlib/server/action_server.h>
#include <actionlib/server/simple_action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group.h>

class HermesVirtualRobot
{

private:
	sensor_msgs::JointState jointState_msg;
	ros::Publisher pub_joint_state_command_;
	std::vector <float> q_left;
	std::vector <float> q_right;


protected:
	ros::NodeHandle node_;

public:
	HermesVirtualRobot(ros::NodeHandle &nh);
	void publishJointState();

private:
	void init();

protected:




};

#endif /* HERMES_VIRTUAL_H_ */
