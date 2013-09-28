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
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>
#include <moveit/move_group_interface/move_group.h>
#include "tf_conversions/tf_kdl.h"
#include <tf/transform_broadcaster.h>


int main(int argc,char* argv[]) {
	ros::init(argc, argv, "fixed_points", ros::init_options::AnonymousName);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // todo: publish initFrame
    tf::TransformBroadcaster initFrame_broad;




	// Creación de Frame Inicial (de q_rightIni matlab)
	KDL::Frame initFrame(KDL::Rotation::Quaternion(-0.4104,0.5206,-0.6023,0.44479),KDL::Vector(0.5277,-0.9699,1.6358));
	KDL::Frame targetFrame(KDL::Rotation::Quaternion(-0.8212,0.3258,-0.1386,0.4475),KDL::Vector(0.7342,-0.2238,1.3370));
	KDL::Frame tmpFrame;

	KDL::Path_Line* path = new KDL::Path_Line(initFrame,targetFrame,new KDL::RotationalInterpolation_SingleAxis(),1.0,true);



	double s;


	s = path->PathLength();

	geometry_msgs::Pose tmpPose;
	std::vector<geometry_msgs::Pose> vecPose;




		for(double i=0;i<=s;i+=0.1){
			tf::PoseKDLToMsg(path->Pos(i),tmpPose);
			vecPose.push_back(tmpPose);
	}



    std::cout << "Tamaño de la trayectoria: "  << vecPose[vecPose.size()-1] << std::endl;

	// todo: Pasar de KDL::Frames a geometry_msgs::Pose
	/*int points = 5;
	for(double i=0;i<=points;i+=s/points){
		tf::PoseKDLToMsg(path->Pos(i),tmpPose);
		vecPose.push_back(tmpPose);
	}*/
	//tf::PoseKDLToMsg(path->Pos(0),tmpPosetf);


	// Confirmado tenemos el path perfectamente definido para pasar lo a ROS moveit
	move_group_interface::MoveGroup group("r_arm");
	moveit_msgs::RobotTrajectory traj;
	double pathPrecision = 0.0;
	pathPrecision = group.computeCartesianPath(vecPose,0.01,10,traj,false);
	std::cout << "Precision de la trayectoria: " << pathPrecision << std::endl;



	/*
	// Este plan no vale no funciona
	moveit::planning_interface::MoveGroup::Plan plan;
	bool have_plan = false;
	for (int trial=0; have_plan==false && trial<5; ++trial){
		have_plan = group.plan(plan);
	}
	if (have_plan==true)
		//group.execute(plan);
		ROS_INFO("Valid plan found for arm movement.");
	else
		ROS_WARN("No valid plan found for arm movement.");
*/






	while(ros::ok())
	{
		//todo: publish initFrame
		tf::Transform initFrame_tf;
		tf::TransformKDLToTF(initFrame,initFrame_tf);


		//initFrame_broad.sendTransform(tf::StampedTransform(initFrame_tf,ros::Time::now(),"pillar","init_frame"));
		//tf::TransformKDLToTF(targetFrame,initFrame_tf);
		//initFrame_broad.sendTransform(tf::StampedTransform(initFrame_tf,ros::Time::now(),"pillar","target_frame"));






		tf::Transform transform_auxiliar;


		transform_auxiliar.setOrigin(tf::Vector3(vecPose[0].position.x, vecPose[0].position.y, vecPose[0].position.z));
		transform_auxiliar.setRotation(tf::Quaternion( vecPose[0].position.x, vecPose[0].orientation.y, vecPose[0].orientation.z, vecPose[0].orientation.w));
		initFrame_broad.sendTransform(tf::StampedTransform(transform_auxiliar,ros::Time::now(),"pillar","init_frame"));

		transform_auxiliar.setOrigin(tf::Vector3(vecPose[vecPose.size()-1].position.x, vecPose[vecPose.size()-1].position.y, vecPose[vecPose.size()-1].position.z));
		transform_auxiliar.setRotation(tf::Quaternion( vecPose[vecPose.size()-1].position.x, vecPose[vecPose.size()-1].orientation.y, vecPose[vecPose.size()-1].orientation.z, vecPose[vecPose.size()-1].orientation.w));
		initFrame_broad.sendTransform(tf::StampedTransform(transform_auxiliar,ros::Time::now(),"pillar","target_frame"));

		//transform_auxiliar.setOrigin(tf::Vector3(tmpPosetf.position.x, tmpPosetf.position.y, tmpPosetf.position.z));
		//transform_auxiliar.setRotation(tf::Quaternion(tmpPosetf.position.x, tmpPosetf.orientation.y, tmpPosetf.orientation.z, tmpPosetf.orientation.w));

		//transform_auxiliar.setOrigin(tf::Vector3(tmpPosetf.position.x, tmpPosetf.position.y, tmpPosetf.position.z));
		//transform_auxiliar.setRotation(tf::Quaternion(tmpPosetf.position.x, tmpPosetf.orientation.y, tmpPosetf.orientation.z, tmpPosetf.orientation.w));
		//initFrame_broad.sendTransform(tf::StampedTransform(transform_auxiliar,ros::Time::now(),"pillar","init_frame"));

		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::waitForShutdown();
	return 0;
}



