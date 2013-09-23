#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <sstream>
#include <string>
#include <hermes_robot_interface/hermes_robot_interface.h>



int main(int argc, char **argv)
{


	ros::init(argc, argv, "hermes_robot_interface");
	ros::NodeHandle n;

	ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);


	HermesRobotInterface hermes(n);
	hermes.init();

	sensor_msgs::JointState joint_msg;
	joint_msg.name.resize(36);
	joint_msg.position.resize(36);
	joint_msg.velocity.resize(36);
	joint_msg.effort.resize(36);


	ros::Rate loop_rate(100);




	while (ros::ok)
	{
		hermes.getJointState(joint_msg);
		joint_state_pub.publish(joint_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
