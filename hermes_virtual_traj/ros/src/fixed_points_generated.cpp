#include "ros/ros.h"
#include "std_msgs/String.h"


#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>
#include <moveit/move_group_interface/move_group.h>


int main(int argc,char* argv[]) {
	ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
    ros::AsyncSpinner spinner(1);
    spinner.start();

	try{
			// Creación de Frame Inicial (de q_rightIni matlab)
			KDL::Frame initFrame(KDL::Rotation::Quaternion(0.0243,0.7940,-0.0578,-0.647),KDL::Vector(0.5277,-0.1258,-0.8949));
			KDL::Frame targetFrame(KDL::Rotation::Quaternion(0.8971,0.2642,-0.3284,-0.1324),KDL::Vector(0.7342,0.1730,-0.1488));
			KDL::Frame tmpFrame;

			KDL::Path_Line* path = new KDL::Path_Line(initFrame,targetFrame,new KDL::RotationalInterpolation_SingleAxis(),1.0,true);

			// Confirmado tenemos el path perfectamente definido para pasar lo a ROS moveit




	}
	catch(KDL::Error& error) {
			std::cout <<"I encountered this error : " << error.Description() << std::endl;
			std::cout << "with the following type " << error.GetType() << std::endl;

	}

	return 0;
}



