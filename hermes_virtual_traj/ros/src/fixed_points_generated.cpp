#include "ros/ros.h"
#include "std_msgs/String.h"
#include <kdl/frames.hpp>


int main(int argc,char* argv[]) {

	ros::init(argc, argv, "target_frames");
	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1);
	ros::Rate loop_rate(10);



	//todo: generar punto inicial y final
	KDL::Frame(KDL::Rotation::RPY(M_PI,0,0), KDL::Vector(1,0,0));


	while (ros::ok())
	{

		std_msgs::String msg;
		std::stringstream ss;
		msg.data = ss.str();
		chatter_pub.publish(msg);

		msg.data = ss.str();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}



