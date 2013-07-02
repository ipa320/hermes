/*
 * camera_calibration.h
 *
 *  Created on: 02.07.2013
 *      Author: rbormann
 */

#ifndef CAMERA_CALIBRATION_H_
#define CAMERA_CALIBRATION_H_

// ROS includes
#include <ros/ros.h>
//#include <ros/package.h>

// ROS message includes
#include <cob_object_detection_msgs/DetectionArray.h>

class CameraCalibration
{
public:

	CameraCalibration(ros::NodeHandle nh);

	~CameraCalibration();

protected:
	/// callback for the incoming pointcloud data stream
	void inputCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg);

	ros::Subscriber input_marker_detection_sub_;	///< detection of the reference coordinate system

	ros::NodeHandle node_handle_;			///< ROS node handle
};

#endif /* CAMERA_CALIBRATION_H_ */
