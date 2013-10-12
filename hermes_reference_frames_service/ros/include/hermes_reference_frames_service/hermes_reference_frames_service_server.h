/*
 * hermes_reference_frames_service_server.h
 *
 *  Created on: 30/07/2013
 *      Author: vr2user
 */

#ifndef HERMES_REFERENCE_FRAMES_SERVICE_SERVER_H_
#define HERMES_REFERENCE_FRAMES_SERVICE_SERVER_H_

#include "ros/ros.h"

#include <hermes_reference_frames_service/HermesFrame.h>
#include <hermes_reference_frames_service/HermesReferenceFrames.h>
#include <iostream>



class HermesReferenceFramesServiceServer
{
public:
	HermesReferenceFramesServiceServer(ros::NodeHandle nh);
	void init();
	bool returnFrame(hermes_reference_frames_service::HermesFrame::Request &req, hermes_reference_frames_service::HermesFrame::Response &res);

protected:
	ros::NodeHandle node_;
	ros::ServiceServer hermes_reference_frames_service_server_; ///< Service server which accepts requests for squaring a number
	HermesReferenceFrames reference_frames;

};



#endif /* HERMES_REFERENCE_FRAMES_SERVICE_SERVER_H_ */
