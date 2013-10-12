
#include "hermes_move_arm_action/traj_action_server.h"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "traj_action_server");

	ros::NodeHandle node;//("~");


	ros::NodeHandle n;
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);


	TrajActionServer jte(node);
	ros::Rate loop_rate(5);

	// Msgs Declaration
	sensor_msgs::JointState joint_state;
	std::vector <float> q_left;
	std::vector <float> q_right;
	q_left.resize(7,0.0);
	q_right.resize(7,0.0);

	// robot state
	double r_joint1 = 0, r_joint2=M_PI/2, r_joint3 = 0, r_joint4 = 0, r_joint5 = 0, r_joint6 = 0, r_joint7 = 0;
	double l_joint1 = 0, l_joint2=M_PI/2, l_joint3 = 0, l_joint4 = 0, l_joint5 = 0, l_joint6 = 0, l_joint7 = 0;

	double r_thumb_joint1 = 0, r_thumb_joint2 = 0, r_thumb_joint3 = 0;
	double r_index_joint1 = 0, r_index_joint2 = 0;
	double r_middle_joint1 = 0, r_middle_joint2 = 0;
	double r_ring_joint1 = 0, r_ring_joint2 = 0;
	double r_pinky_joint1 = 0, r_pinky_joint2 = 0;

	double l_thumb_joint1 = 0, l_thumb_joint2 = 0, l_thumb_joint3 = 0;
	double l_index_joint1 = 0, l_index_joint2 = 0;
	double l_middle_joint1 = 0, l_middle_joint2 = 0;
	double l_ring_joint1 = 0, l_ring_joint2 = 0;
	double l_pinky_joint1 = 0, l_pinky_joint2 = 0;

	joint_state.header.stamp = ros::Time::now();
	joint_state.name.resize(36);
	joint_state.position.resize(36);
	joint_state.name[0] ="r_joint1";
	joint_state.name[1] ="r_joint2";
	joint_state.name[2] ="r_joint3";
	joint_state.name[3] ="r_joint4";
	joint_state.name[4] ="r_joint5";
	joint_state.name[5] ="r_joint6";
	joint_state.name[6] ="r_joint7";
	joint_state.name[7] ="l_joint1";
	joint_state.name[8] ="l_joint2";
	joint_state.name[9] ="l_joint3";
	joint_state.name[10] ="l_joint4";
	joint_state.name[11] ="l_joint5";
	joint_state.name[12] ="l_joint6";
	joint_state.name[13] ="l_joint7";
	joint_state.name[14] ="r_thumb_joint1";
	joint_state.name[15] ="r_thumb_joint2";
	joint_state.name[16] ="r_thumb_joint3";
	joint_state.name[17] ="r_index_joint1";
	joint_state.name[18] ="r_index_joint2";
	joint_state.name[19] ="r_middle_joint1";
	joint_state.name[20] ="r_middle_joint2";
	joint_state.name[21] ="r_ring_joint1";
	joint_state.name[22] ="r_ring_joint2";
	joint_state.name[23] ="r_pinky_joint1";
	joint_state.name[24] ="r_pinky_joint2";
	joint_state.name[25] ="l_thumb_joint1";
	joint_state.name[26] ="l_thumb_joint2";
	joint_state.name[27] ="l_thumb_joint3";
	joint_state.name[28] ="l_index_joint1";
	joint_state.name[29] ="l_index_joint2";
	joint_state.name[30] ="l_middle_joint1";
	joint_state.name[31] ="l_middle_joint2";
	joint_state.name[32] ="l_ring_joint1";
	joint_state.name[33] ="l_ring_joint2";
	joint_state.name[34] ="l_pinky_joint1";
	joint_state.name[35] ="l_pinky_joint2";


	//while (ros::ok()) {
		//std::cout << "Publicando Joint State" << std::endl;
		jte.getJointStateRightArm(q_right);

		std::cout << "Joint 0: " << q_right[0] << std::endl;
		joint_state.position[0] = q_right[0];
		joint_state.position[1] = q_right[1];
		joint_state.position[2] = q_right[2];
		joint_state.position[3] = q_right[3];
		joint_state.position[4] = q_right[4];
		joint_state.position[5] = q_right[5];
		joint_state.position[6] = q_right[6];
		joint_state.position[7] = q_left[0];
		joint_state.position[8] = q_left[1];
		joint_state.position[9] = q_left[2];
		joint_state.position[10] = q_left[3];
		joint_state.position[11] = q_left[4];
		joint_state.position[12] = q_left[5];
		joint_state.position[13] = q_left[6];
		joint_state.position[14] = r_thumb_joint1;
		joint_state.position[15] = r_thumb_joint2;
		joint_state.position[16] = r_thumb_joint3;
		joint_state.position[17] = r_index_joint1;
		joint_state.position[18] = r_index_joint2;
		joint_state.position[19] = r_middle_joint1;
		joint_state.position[20] = r_middle_joint2;
		joint_state.position[21] = r_ring_joint1;
		joint_state.position[22] = r_ring_joint2;
		joint_state.position[23] = r_pinky_joint1;
		joint_state.position[24] = r_pinky_joint2;
		joint_state.position[25] = l_thumb_joint1;
		joint_state.position[26] = l_thumb_joint2;
		joint_state.position[27] = l_thumb_joint3;
		joint_state.position[28] = l_index_joint1;
		joint_state.position[29] = l_index_joint2;
		joint_state.position[30] = l_middle_joint1;
		joint_state.position[31] = l_middle_joint2;
		joint_state.position[32] = l_ring_joint1;
		joint_state.position[33] = l_ring_joint2;
		joint_state.position[34] = l_pinky_joint1;
		joint_state.position[35] = l_pinky_joint2;

		joint_pub.publish(joint_state);
		//loop_rate.sleep();
	//}

		ros::spin();
	return 0;
}



