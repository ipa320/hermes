#ifndef HERMES_GRASP_DATABASE_H_
#define HERMES_GRASP_DATABASE_H_

// ROS includes
#include "ros/ros.h"
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <hermes_grasp_database/HermesGraspDatabaseConfig.h>

// services - here you have to include the header file with exactly the same name as your message in the /srv folder (the Message.h is automatically generated from your Message.srv file during compilation)
#include <hermes_grasp_database/GetGraspForDetection.h>

#include <map>
#include <vector>
#include <string>

struct GraspDatabaseEntry
{
	std::string label;			// object name
	tf::Transform grasp_offset;	// offset of wrist joint to object center that is suitable for grasping
	int grasp_type;				// type of best suited grasp for this object
	int grasp_force;			// force applied to the grasp [0,..,100]
};

class HermesGraspDatabase
{
public:
	HermesGraspDatabase(ros::NodeHandle nh);
	void init(std::string databaseFile);

protected:
	void loadDatabase(std::string databaseFile);
	bool computeGraspForDetection(hermes_grasp_database::GetGraspForDetection::Request &req, hermes_grasp_database::GetGraspForDetection::Response &res);

	void dynamicReconfigureCallback(hermes_grasp_database::HermesGraspDatabaseConfig &config, uint32_t level);

	ros::NodeHandle node_;
	ros::ServiceServer hermes_grasp_database_service_server_; ///< Service server which accepts requests

	dynamic_reconfigure::Server<hermes_grasp_database::HermesGraspDatabaseConfig> dynamic_reconfigure_server_;

	tf::TransformListener transform_listener_;

	///     object label, vector of grasps
	std::map<std::string, std::vector<GraspDatabaseEntry> > grasp_database_;

	std::string target_frame_id_;		///< the name of the target frame in whose coordinates the pose is provided in computeGraspForDetection
};

#endif /* HERMES_GRASP_DATABASE_H_ */
