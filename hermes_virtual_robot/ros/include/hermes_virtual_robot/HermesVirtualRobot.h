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
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/callback_queue.h>
#include "hermes_virtual_robot/HermesJointStates.h"
#include "hermes_virtual_robot/GraspHand.h"



class HermesVirtualRobot
{
	protected:
		ros::NodeHandle node_;

	private:
		bool hermes_correct;  //Indicates if hermes virtual robot is in correct state
		urdf::Model hermes_model;   // model of the robot
		bool is_publish;


	private:
		typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JTAS;
		typedef JTAS::GoalHandle GoalHandle;
		JTAS action_server_right_arm_;
		JTAS action_server_left_arm_;
		GoalHandle active_goal_right_;
		GoalHandle active_goal_left_;
		trajectory_msgs::JointTrajectory current_traj_right_;
		trajectory_msgs::JointTrajectory current_traj_left_;
		ros::Publisher pub_controller_command_left_;
		ros::Publisher pub_controller_command_right_;

		ros::ServiceServer joint_states_service_;
		ros::ServiceServer grasp_hand_;




	public:
		HermesVirtualRobot(ros::NodeHandle);
		bool getHermesCorrect();
		void publish_robot_state();
		void executeTrajRightArmCB(GoalHandle gh);
		void executeTrajLeftArmCB(GoalHandle gh);

	private:


		void moveVirtualRightArm();
		void moveVirtualLeftArm();

		bool getJointStateServer(hermes_virtual_robot::HermesJointStates::Request  &req, hermes_virtual_robot::HermesJointStates::Response &res);
		bool graspCB(hermes_virtual_robot::GraspHand::Request  &req, hermes_virtual_robot::GraspHand::Response &res);


	private:
		void readUrdfFile();

	protected:


		tf::TransformBroadcaster broadcaster; //tf_broadcaster for joint_states
		robot_model_loader::RobotModelLoader robot_model_loader_;
		robot_model::RobotModelPtr kinematic_model_;
		robot_model::RobotModel *robot_model_;
		robot_state::RobotStatePtr kinematic_state_;
		robot_state::JointStateGroup* joint_state_group_right_arm_;
		robot_state::JointStateGroup* joint_state_group_left_arm_;
		sensor_msgs::JointState joint_msg_;
		sensor_msgs::JointState joint_msg_left;
		sensor_msgs::JointState joint_msg_right;
		std::map< std::string, double > joint_state_values;




	private:
		ros::CallbackQueue move_right_arm_queue;











};


#endif /* HERMESVIRTUALROBOT_H_ */

