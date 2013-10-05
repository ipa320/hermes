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
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <actionlib/server/action_server.h>
#include <actionlib/server/simple_action_server.h>
#include <hermes_virtual_robot/MoveArmAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>



class HermesVirtualRobot
{
	private:
		bool hermes_correct;  //Indicates if hermes virtual robot is in correct state
		urdf::Model hermes_model;   // model of the robot


	private:
		typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JTAS;
		typedef JTAS::GoalHandle GoalHandle;
		JTAS action_server_;
		GoalHandle active_goal_;

	public:
		HermesVirtualRobot(ros::NodeHandle);
		bool getHermesCorrect();
		void publish_robot_state();
		void executeTrajCB(GoalHandle gh);

	private:
		void readUrdfFile();

	protected:

		ros::NodeHandle node_;
		tf::TransformBroadcaster broadcaster; //tf_broadcaster for joint_states
		ros::Publisher joint_pub; // Publish JointState
		robot_model_loader::RobotModelLoader robot_model_loader_;
		robot_model::RobotModelPtr kinematic_model_;
		robot_model::RobotModel *robot_model_;
		robot_state::RobotStatePtr kinematic_state_;
		robot_state::JointStateGroup* joint_state_group_right_arm_;
		robot_state::JointStateGroup* joint_state_group_left_arm_;
		sensor_msgs::JointState joint_msg_;
		std::map< std::string, double > joint_state_values;













};


#endif /* HERMESVIRTUALROBOT_H_ */

