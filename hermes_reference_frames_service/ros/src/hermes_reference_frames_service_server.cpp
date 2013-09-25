/*
 * hermes_reference_frames_service.cpp
 *
 *  Created on: 30/07/2013
 *      Author: vr2user
 */

#include "hermes_reference_frames_service/hermes_reference_frames_service_server.h"

HermesReferenceFramesServiceServer::HermesReferenceFramesServiceServer(ros::NodeHandle nh)
{
	node_ = nh;
}


void HermesReferenceFramesServiceServer::init()
{
	/**
	* The advertiseService() function is how you tell ROS that you want to provide a service for other modules (software nodes).
	*/

	hermes_reference_frames_service_server_ = node_.advertiseService("reference_frames_service", &HermesReferenceFramesServiceServer::returnFrame, this);

//	left_hand_.init(1);
//	right_hand_.init(2);
}


bool HermesReferenceFramesServiceServer::returnFrame(hermes_reference_frames_service::HermesFrame::Request &req, hermes_reference_frames_service::HermesFrame::Response &res)
{
	// this callback function is executed each time a request comes in for this service server
	// here we just read the number from the request, square it and put the result into the response, the response is automatically sent back to the caller when this function returns

	ROS_INFO("HermesRefenceFrames Service Server: Received a request with frame %i.",req.frame);


	tf::Transform *tf = new tf::Transform(tf::Quaternion(0,0,0,0),tf::Vector3(0,0,0));



	if (req.frame == hermes_reference_frames_service::HermesFrame::Request::WORLDTLEFTARM){
		reference_frames.getworldTleftarm(tf);
	}

	else if (req.frame == hermes_reference_frames_service::HermesFrame::Request::WORLDTRIGHTARM){

		reference_frames.getworldTrightarm(tf);
	}

	else
	{
		res.message = "wrong hand specified.";
		return false;
	}

	// Pass to response

	res.position[0] = tf->getOrigin().getX();
	res.position[1] = tf->getOrigin().getY();
	res.position[2] = tf->getOrigin().getZ();


	res.quaternion[0] = tf->getRotation().getX();
	res.quaternion[1] = tf->getRotation().getY();
	res.quaternion[2] = tf->getRotation().getZ();
	res.quaternion[3] = tf->getRotation().getW();


	delete tf;


	// if the procedure fails, use "return false;" to inform the caller of the service
	res.message = "success";
	return true;	// tell the caller that the method was successfully executed
}



