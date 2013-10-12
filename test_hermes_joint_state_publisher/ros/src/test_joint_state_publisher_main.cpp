#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <sstream>
#include <string>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_hermes_joint_state");
	ros::NodeHandle n;

	ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("hermes_joint_state", 1000);


	// Creación del joint State
	sensor_msgs::JointState joint_msg;

	joint_msg.name.resize(1);
	joint_msg.position.resize(1);
	joint_msg.velocity.resize(1);
	joint_msg.effort.resize(1);

	joint_msg.name[0] = "r_link1";
	joint_msg.position[0] = 0.0;

	ros::Rate loop_rate(10);

	while (ros::ok)
	{
		joint_msg.position[0] += 0.01;

		 joint_state_pub.publish(joint_msg);

		ros::spinOnce();
		loop_rate.sleep();

	}
}
