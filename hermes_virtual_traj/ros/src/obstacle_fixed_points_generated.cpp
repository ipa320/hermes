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
#include <tf/transform_listener.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>



int main(int argc,char* argv[]) {
	ros::init(argc, argv, "fixed_points", ros::init_options::AnonymousName);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // todo: publish initFrame
//    tf::TransformBroadcaster initFrame_broad;



    //todo: attach object to robot for collisions
    boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(2.0)));
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf));

    ros::Publisher attached_object_publisher = n.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1);
    while(attached_object_publisher.getNumSubscribers() < 1)
    {
      ros::WallDuration sleep_t(0.5);
      sleep_t.sleep();
    }

    //todo: create object
    moveit_msgs::AttachedCollisionObject attached_robot_object;
    attached_robot_object.link_name = "r_base_hand";
    attached_robot_object.object.header.frame_id = "r_eef";
    attached_robot_object.object.id = "box";
    geometry_msgs::Pose pose_robot_object;
    pose_robot_object.orientation.w = 1.0;
    pose_robot_object.position.x = 0.0;
    pose_robot_object.position.y = 0.15;
    pose_robot_object.position.z = 0.0;
    shape_msgs::SolidPrimitive primitive_robot;
    primitive_robot.type = primitive_robot.BOX;
    primitive_robot.dimensions.resize(3);
    primitive_robot.dimensions[0] = 0.2;
    primitive_robot.dimensions[1] = 0.2;
    primitive_robot.dimensions[2] = 0.2;
    attached_robot_object.object.primitives.push_back(primitive_robot);
    attached_robot_object.object.primitive_poses.push_back(pose_robot_object);
    attached_robot_object.object.operation = attached_robot_object.object.ADD;
    attached_object_publisher.publish(attached_robot_object);



    //Get package path
    std::string path = ros::package::getPath("hermes_scenario");
    path += "/common/files/stl/";



    //todo: add collsions objects into the environment
    moveit_msgs::AttachedCollisionObject big_table;

    big_table.object.id="big_table";
    big_table.object.header.stamp = ros::Time::now();
    big_table.object.header.frame_id = "pillar";
    big_table.object.operation = moveit_msgs::CollisionObject::ADD;
    big_table.object.primitives.resize(1);
   // big_table.object.primitives[0].type = shape_msgs::SolidPrimitive::MESH;
    //big_table.object.primitives[0].type = shape_msgs::Mesh kk;
    big_table.object.primitives[0].dimensions.push_back(0.1);
    big_table.object.primitives[0].dimensions.push_back(0.1);
    big_table.object.primitives[0].dimensions.push_back(0.4);
    big_table.object.primitive_poses.resize(1);
    big_table.object.primitive_poses[0].position.x = 0.1;
    big_table.object.primitive_poses[0].position.y = 0;
    big_table.object.primitive_poses[0].position.z = -0.2;
    big_table.object.primitive_poses[0].orientation.w = 1.0;

	geometry_msgs::Pose pose_environtment;
	pose_environtment.position.x = 1.0;
	pose_environtment.position.y = 0.0;
	pose_environtment.position.z = 0.0;
	pose_environtment.orientation.w = 1.0;



	//shape_msgs::Mesh mesh_big_table;
	//mesh_big_table.       EigenSTL::vector_Vector3d &source
	/*shape_msgs::SolidPrimitive primitive_big_table;
	primitive_big_table.type = primitive_big_table.BOX;
	primitive_big_table.dimensions.resize(3);
	primitive_big_table.dimensions[0] = 0.2;
	primitive_big_table.dimensions[1] = 0.2;
	primitive_big_table.dimensions[2] = 0.2;
	attached_object_big_table.object.primitives.push_back(primitive_big_table);
	attached_object_big_table.object.primitive_poses.push_back(pose_environtment);
	attached_object_big_table.object.operation = attached_object_big_table.object.ADD;*/








	ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	while(planning_scene_diff_publisher.getNumSubscribers() < 1)
	{
	  ros::WallDuration sleep_t(0.5);
	  sleep_t.sleep();
	}

	moveit_msgs::PlanningScene planning_scene;
	planning_scene.world.collision_objects.push_back(big_table.object);
	planning_scene.is_diff = true;
	planning_scene_diff_publisher.publish(planning_scene);




	// Creación de Frame Inicial (de q_rightIni matlab)
//	KDL::Frame initFrame(KDL::Rotation::Quaternion(-0.4104,0.5206,-0.6023,0.44479),KDL::Vector(0.5277,-0.9699,1.6358));
//	KDL::Frame targetFrame(KDL::Rotation::Quaternion(-0.8212,0.3258,-0.1386,0.4475),KDL::Vector(0.7342,-0.2238,1.3370));
//	KDL::Frame tmpFrame;

//	KDL::Path_Line* path = new KDL::Path_Line(initFrame,targetFrame,new KDL::RotationalInterpolation_SingleAxis(),1.0,true);



//	double s;


//	s = path->PathLength();

//	geometry_msgs::Pose tmpPose;
//	std::vector<geometry_msgs::Pose> vecPose;




//		for(double i=0;i<=s;i+=0.1){
//			tf::PoseKDLToMsg(path->Pos(i),tmpPose);
//			vecPose.push_back(tmpPose);
//	}







	// Confirmado tenemos el path perfectamente definido para pasar lo a ROS moveit
//	move_group_interface::MoveGroup group("r_arm");
//	moveit_msgs::RobotTrajectory traj;
//	double pathPrecision = 0.0;
//	pathPrecision = group.computeCartesianPath(vecPose,0.01,10,traj,false);
//	std::cout << "Precision de la trayectoria: " << pathPrecision << std::endl;










	while(ros::ok())
	{
		//todo: publish initFrame
//		tf::Transform initFrame_tf;
//		tf::TransformKDLToTF(initFrame,initFrame_tf);


//		tf::Transform transform_auxiliar;


//		transform_auxiliar.setOrigin(tf::Vector3(vecPose[0].position.x, vecPose[0].position.y, vecPose[0].position.z));
//		transform_auxiliar.setRotation(tf::Quaternion( vecPose[0].orientation.x, vecPose[0].orientation.y, vecPose[0].orientation.z, vecPose[0].orientation.w));
//		initFrame_broad.sendTransform(tf::StampedTransform(transform_auxiliar,ros::Time::now(),"pillar","init_frame"));




//		transform_auxiliar.setOrigin(tf::Vector3(vecPose[vecPose.size()-1].position.x, vecPose[vecPose.size()-1].position.y, vecPose[vecPose.size()-1].position.z));
//		transform_auxiliar.setRotation(tf::Quaternion( vecPose[vecPose.size()-1].orientation.x, vecPose[vecPose.size()-1].orientation.y, vecPose[vecPose.size()-1].orientation.z, vecPose[vecPose.size()-1].orientation.w));
//		initFrame_broad.sendTransform(tf::StampedTransform(transform_auxiliar,ros::Time::now(),"pillar","target_frame"));

		//transform_auxiliar.setOrigin(tf::Vector3(tmpPosetf.position.x, tmpPosetf.position.y, tmpPosetf.position.z));
		//transform_auxiliar.setRotation(tf::Quaternion(tmpPosetf.position.x, tmpPosetf.orientation.y, tmpPosetf.orientation.z, tmpPosetf.orientation.w));


		//transform_auxiliar.setOrigin(tf::Vector3(tmpPosetf.position.x, tmpPosetf.position.y, tmpPosetf.position.z));
		//transform_auxiliar.setRotation(tf::Quaternion(tmpPosetf.position.x, tmpPosetf.orientation.y, tmpPosetf.orientation.z, tmpPosetf.orientation.w));
		//initFrame_broad.sendTransform(tf::StampedTransform(transform_auxiliar,ros::Time::now(),"pillar","init_frame"));


		//todo: collisions objects
		//attached_object_publisher.publish(attached_object);


		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::waitForShutdown();
	return 0;
}



