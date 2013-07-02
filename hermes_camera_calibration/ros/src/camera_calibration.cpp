#include <hermes_camera_calibration/camera_calibration.h>

CameraCalibration::CameraCalibration(ros::NodeHandle nh)
: node_handle_(nh)
{
	// subscribers
	input_marker_detection_sub_ = node_handle_.subscribe("input_marker_detections", 1, &CameraCalibration::inputCallback, this);
}

CameraCalibration::~CameraCalibration()
{
}

/// callback for the incoming pointcloud data stream
void CameraCalibration::inputCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg)
{
	std::cout << "Receiving detection of coordinate system..." << std::endl;

	// search for marker No. 1
	for (unsigned int i=0; i<input_marker_detections_msg->detections.size(); i++)
	{
		if (input_marker_detections_msg->detections[i].label.compare("1") == true)
		{
			// update average coordinate system of marker

			// publish new coordinate transformation
		}
	}
}


int main (int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "camera_calibration");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create and initialize an instance of CameraCalibration
	CameraCalibration objectRecording(nh);

	ros::spin();

	return (0);
}
