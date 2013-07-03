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
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <hermes_camera_calibration/CameraCalibrationConfig.h>

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

	/// dynamic reconfigure callback
	void dynamicReconfigureCallback(hermes_camera_calibration::CameraCalibrationConfig &config, uint32_t level);

	ros::Subscriber input_marker_detection_sub_;	///< detection of the reference coordinate system

	tf::Vector3 base_translation_;
	tf::Quaternion base_orientation_;
	double update_rate_;
	std::string parent_frame_for_camera_;

	tf::TransformBroadcaster transform_broadcaster_;
	tf::TransformListener transform_listener_;

	dynamic_reconfigure::Server<hermes_camera_calibration::CameraCalibrationConfig> dynamic_reconfigure_server_;

	ros::NodeHandle node_handle_;			///< ROS node handle
};

#endif /* CAMERA_CALIBRATION_H_ */
