#include "hermes_virtual_traj/hermes_virtual_robot.h"

HermesVirtualRobot::HermesVirtualRobot(ros::NodeHandle &nh): node_(nh)
{
	init();
	pub_joint_state_command_ =
		    	      node_.advertise<sensor_msgs::JointState>("joint_states", 1);
}

void HermesVirtualRobot::init()
{

	// Resize q_left and q_right vectors
	q_left.resize(7);
	q_right.resize(7);

	// Init position of virtual robot
	q_left[0] =  0.1935;
	q_left[1] =  1.0232;
	q_left[2] =  2.7270;
	q_left[3] = -1.3565;
	q_left[4] =  0.4324;
	q_left[5] = -0.1122;
	q_left[6] = -3.0668;

	q_right[0] =  0.7291;
	q_right[1] = -0.0979;
	q_right[2] = -0.9321;
	q_right[3] =  1.2126;
	q_right[4] =  0.5357;
	q_right[5] =  0.1822;
	q_right[6] =  2.6213;


	 //todo:  JointState inicialize
	jointState_msg.name.resize(36);
	jointState_msg.position.resize(36);
	jointState_msg.velocity.resize(36);
	jointState_msg.effort.resize(36);


	//todo: Put name to JointState
	jointState_msg.name[0] ="r_joint1";
	jointState_msg.name[1] ="r_joint2";
	jointState_msg.name[2] ="r_joint3";
	jointState_msg.name[3] ="r_joint4";
	jointState_msg.name[4] ="r_joint5";
	jointState_msg.name[5] ="r_joint6";
	jointState_msg.name[6] ="r_joint7";
	jointState_msg.name[7] ="l_joint1";
	jointState_msg.name[8] ="l_joint2";
	jointState_msg.name[9] ="l_joint3";
	jointState_msg.name[10] ="l_joint4";
	jointState_msg.name[11] ="l_joint5";
	jointState_msg.name[12] ="l_joint6";
	jointState_msg.name[13] ="l_joint7";
	jointState_msg.name[14] ="r_thumb_joint1";
	jointState_msg.name[15] ="r_thumb_joint2";
	jointState_msg.name[16] ="r_thumb_joint3";
	jointState_msg.name[17] ="r_index_joint1";
	jointState_msg.name[18] ="r_index_joint2";
	jointState_msg.name[19] ="r_middle_joint1";
	jointState_msg.name[20] ="r_middle_joint2";
	jointState_msg.name[21] ="r_ring_joint1";
	jointState_msg.name[22] ="r_ring_joint2";
	jointState_msg.name[23] ="r_pinky_joint1";
	jointState_msg.name[24] ="r_pinky_joint2";
	jointState_msg.name[25] ="l_thumb_joint1";
	jointState_msg.name[26] ="l_thumb_joint2";
	jointState_msg.name[27] ="l_thumb_joint3";
	jointState_msg.name[28] ="l_index_joint1";
	jointState_msg.name[29] ="l_index_joint2";
	jointState_msg.name[30] ="l_middle_joint1";
	jointState_msg.name[31] ="l_middle_joint2";
	jointState_msg.name[32] ="l_ring_joint1";
	jointState_msg.name[33] ="l_ring_joint2";
	jointState_msg.name[34] ="l_pinky_joint1";
	jointState_msg.name[35] ="l_pinky_joint2";
}

void HermesVirtualRobot::publishJointState()
{
	jointState_msg.header.stamp =ros::Time::now();
	for(int i=0;i<7;++i){
		jointState_msg.position[i] = q_right[i];
		jointState_msg.position[i+7] = q_left[i];
	}
	pub_joint_state_command_.publish(jointState_msg);
}

