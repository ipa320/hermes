#include "hermes_move_arm_action/hermes_move_arm_action_server.h"
#include "hermes_arm_kdl/ikine.h"
#include "hermes_reference_frames_service/HermesFrame.h"



HermesMoveArmActionServer::HermesMoveArmActionServer(ros::NodeHandle nh)
: move_arm_action_server_(nh, "move_arm_action", boost::bind(&HermesMoveArmActionServer::moveArm, this, _1), false)	// this initializes the action server; important: always set the last parameter to false
{
	node_ = nh;
}


void HermesMoveArmActionServer::init()
{
	std::cout << "Waiting for service server to become available..." << std::endl;
	bool serviceAvailable = ros::service::waitForService("/arm_kdl_service_ikine_server", 5000);

	// only proceed if the service is available
	if (serviceAvailable == false)
	{
		std::cout << "The service could not be found.\n" << std::endl;
		return;
	}
	std::cout << "The service server is advertised.\n" << std::endl;

	// by starting the action server, your action gets advertised to other software modules

	move_arm_action_server_.start();

	// todo:  bring-up robot modules
	hermesinterface.init();




}


void HermesMoveArmActionServer::moveArm(const hermes_move_arm_action::MoveArmGoalConstPtr& goal)
{
	// this callback function is executed each time a request (= goal message) comes in for this service server
	ROS_INFO("MoveArm Action Server: Received a request for arm %i.", goal->arm);

	// this command sends a feedback message to the caller, here we transfer that the task is completed 25%
//	hermes_move_arm_action::MoveArmFeedback feedback;
//	feedback.percentageDone = 25;
//	move_arm_action_server_.publishFeedback(feedback);

	// move the arm
	// ...

//goal->goal_position.pose.position.x  //(x,y,z) in meters?????
//goal->goal_position.pose.orientation.w //(w,x,y,z) Quaternion in rad
//goal->goal_position.header.frame_id // Reference frame
	tf::Quaternion quaternion;
	tf::quaternionMsgToTF(goal->goal_position.pose.orientation, quaternion);
	tf::Transform goalPos(quaternion, tf::Vector3(goal->goal_position.pose.position.x,goal->goal_position.pose.position.y,goal->goal_position.pose.position.z));

	std::cout << "goalPos: " << std::endl;
    std::cout << goalPos.getRotation().getW() <<std::endl;
    std::cout << goalPos.getRotation().getX() <<std::endl;
    std::cout << goalPos.getRotation().getY() <<std::endl;
    std::cout << goalPos.getRotation().getZ() <<std::endl;
    std::cout << "XYZ: " << std::endl;
	std::cout << goalPos.getOrigin().getX() <<std::endl;
	std::cout << goalPos.getOrigin().getY() <<std::endl;
	std::cout << goalPos.getOrigin().getZ() <<std::endl;


	// Transform goalPos to Robot Reference frame
	hermes_reference_frames_service::HermesFrame::Request req_frames;
	hermes_reference_frames_service::HermesFrame::Response res_frames;

	if(goal->arm == hermes_move_arm_action::MoveArmGoal::LEFTARM) // Depends of arm
		req_frames.frame = hermes_reference_frames_service::HermesFrame::Request::WORLDTLEFTARM;

	else if(goal->arm == hermes_move_arm_action::MoveArmGoal::RIGHTARM) // Depends of arm
		req_frames.frame = hermes_reference_frames_service::HermesFrame::Request::WORLDTRIGHTARM;

	ros::service::call("/reference_frames_service", req_frames, res_frames);

	// Transform world to base robot
	tf::Quaternion qua_wTr(res_frames.quaternion[0],res_frames.quaternion[1],res_frames.quaternion[2],res_frames.quaternion[3]);
	tf::Transform wTr(qua_wTr, tf::Vector3(res_frames.position[0],res_frames.position[1],res_frames.position[2]));

	tf::Transform rTobj;
	rTobj = wTr.inverse()*goalPos;

	std::cout << "goalPosNew: " << std::endl;
	std::cout << rTobj.getRotation().getW() <<std::endl;
	std::cout << rTobj.getRotation().getX() <<std::endl;
	std::cout << rTobj.getRotation().getY() <<std::endl;
	std::cout << rTobj.getRotation().getZ() <<std::endl;

	std::cout << "XYZ: " << std::endl;
	std::cout << rTobj.getOrigin().getX() <<std::endl;
	std::cout << rTobj.getOrigin().getY() <<std::endl;
	std::cout << rTobj.getOrigin().getZ() <<std::endl;


	tf::Vector3 rotX=rTobj.getBasis()[0];
	tf::Vector3 rotY=rTobj.getBasis()[1];
	tf::Vector3 rotZ=rTobj.getBasis()[2];


	KDL::Vector rotXkdl(rotX.getX(),rotX.getY(),rotX.getZ());
	KDL::Vector rotYkdl(rotY.getX(),rotY.getY(),rotY.getZ());
	KDL::Vector rotZkdl(rotZ.getX(),rotZ.getY(),rotZ.getZ());

	KDL::Rotation rot(rotXkdl,rotYkdl,rotZkdl);



	//Get init position
	std::vector<float> q_init;

	if(goal->arm == hermes_move_arm_action::MoveArmGoal::LEFTARM)
		hermesinterface.get_leftJoints(q_init);
	else if(goal->arm == hermes_move_arm_action::MoveArmGoal::RIGHTARM)
		hermesinterface.get_rightJoints(q_init);


   // todo: call hermes_arm_kdl ikine service
	hermes_arm_kdl::ikine::Request req_kdl;
	hermes_arm_kdl::ikine::Response res_kdl;
	for (int i=0; i<7; i++)
		req_kdl.jointAngles_init[i] = q_init[i];
		req_kdl.position[0] = rTobj.getOrigin().getX();
		req_kdl.position[1] = rTobj.getOrigin().getY();
		req_kdl.position[2] = rTobj.getOrigin().getZ();

	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			req_kdl.rotation[3*i+j] = rot(i,j);
	ros::service::call("/arm_kdl_service_ikine_server", req_kdl, res_kdl);

	// Move the arm with res_kdl
	std::vector<float> jointAngles(7);

	for (int i=0; i<7; i++)
			jointAngles[i] = res_kdl.jointAngles[i];

	//ARMS DON'T MOVE


//	if(goal->arm == hermes_move_arm_action::MoveArmGoal::LEFTARM)
//		hermesinterface.moveLeftArm(jointAngles);
//	else if(goal->arm == hermes_move_arm_action::MoveArmGoal::RIGHTARM)
//		hermesinterface.moveRightArm(jointAngles);

	hermes_move_arm_action::MoveArmResult res;
	res.return_value.val = arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS; 	// put in there some error code on errors


	// this sends the response back to the caller
	move_arm_action_server_.setSucceeded(res);

	// if failed, use: move_arm_action_server_.setAborted(res);
}
