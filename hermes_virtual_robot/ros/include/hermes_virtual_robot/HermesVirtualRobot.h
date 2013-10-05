/*
 * HermesVirtualRobot.h
 *
 *  Created on: 04/10/2013
 *      Author: ricardo
 */

#ifndef HERMESVIRTUALROBOT_H_
#define HERMESVIRTUALROBOT_H_

#include "ros/ros.h"
#include <urdf/model.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

class HermesVirtualRobot
{
	private:
		bool hermes_correct;  //Indicates if hermes virtual robot is in correct state
		urdf::Model hermes_model;   // model of the robot



	public:
		HermesVirtualRobot(ros::NodeHandle);
		bool getHermesCorrect();
		void publish_robot_state();

	private:
		void readUrdfFile();

	protected:

		ros::NodeHandle node_;
		tf::TransformBroadcaster broadcaster; //tf_broadcaster for joint_states
		ros::Publisher joint_pub; // Publish JointState



};


#endif /* HERMESVIRTUALROBOT_H_ */

