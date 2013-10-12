#include "hermes_virtual_robot/HermesVirtualRobot.h"

// Depend packages: urdf
//				    roslib

int main(int argc, char** argv){



	ros::init(argc, argv, "hermes_virtual_robot_publisher");
	ros::NodeHandle n;

	ros::Rate loop_rate(30);


	ros::ServiceClient client = n.serviceClient<hermes_virtual_robot::HermesJointStates>("hermes_joint_states");

	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);



	sensor_msgs::JointState joint_msg;

	hermes_virtual_robot::HermesJointStates srv;



	 while (ros::ok()) {

		 	srv.request.header.stamp = ros::Time::now();

			if (client.call(srv)){
				joint_msg = srv.response.hermes_joint_states;
				joint_msg.header.stamp=ros::Time::now();
				joint_msg.header.frame_id="/world";
				joint_msg.velocity.resize(joint_msg.name.size());
				joint_msg.effort.resize(joint_msg.name.size());
				joint_pub.publish(joint_msg);
			}
			ros::spinOnce();
			loop_rate.sleep();
	  }

	  return 0;



}




