#include "ros/ros.h"

#include "tf/tf.h"

// services - here you have to include the header file with exactly the same name as your message in the /srv folder (the Message.h is automatically generated from your Message.srv file during compilation)
#include <hermes_grasp_database/GetGraspForDetection.h>

struct GraspDatabaseEntry
{
	std::string label;			// object name
	tf::Transform grasp_offset;	// offset of wrist joint to object center that is suitable for grasping
	int grasp_type;				// type of best suited grasp for this object
};

class HermesGraspDatabase
{
public:
	HermesGraspDatabase(ros::NodeHandle nh);
	void init(std::string databaseFile);

protected:
	void loadDatabase(std::string databaseFile);
	bool computeGraspForDetection(hermes_grasp_database::GetGraspForDetection::Request &req, hermes_grasp_database::GetGraspForDetection::Response &res);

	ros::NodeHandle node_;
	ros::ServiceServer hermes_grasp_database_service_server_; ///< Service server which accepts requests


};
