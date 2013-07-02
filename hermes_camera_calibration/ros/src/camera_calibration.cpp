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
