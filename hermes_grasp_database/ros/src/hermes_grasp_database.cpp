#include "hermes_grasp_database/hermes_grasp_database.h"

#include "hermes_grasp_database/GraspConfiguration.h"
#include "geometry_msgs/PoseStamped.h"

#include <fstream>


HermesGraspDatabase::HermesGraspDatabase(ros::NodeHandle nh)
{
	node_ = nh;
}


void HermesGraspDatabase::init(std::string databaseFile)
{
	loadDatabase(databaseFile);

	/**
	* The advertiseService() function is how you tell ROS that you want to provide a service for other modules (software nodes).
	*/
	hermes_grasp_database_service_server_ = node_.advertiseService("get_grasp_for_detection", &HermesGraspDatabase::computeGraspForDetection, this);
}


void HermesGraspDatabase::loadDatabase(std::string databaseFile)
{
	std::ifstream file(databaseFile.c_str(), std::ios::in);
	if (file.is_open() == false)
	{
		std::cout << "Error: could not open file '" << databaseFile << "'." << std::endl;
		return;
	}
	else
		std::cout << "\nReading database...\n\n";

	bool have_object_label = false, have_grasp_type = false, have_grasp_force = false, have_grasp_offset_position = false, have_grasp_offset_quaternion = false;
	GraspDatabaseEntry gde;
	tf::Quaternion q;
	tf::Vector3 t;
	while (file.eof() == false)
	{
		std::string tag;
		file >> tag;
		//std::cout << "tag=" << tag << std::endl;

		if (tag.compare("object_label:") == 0)
		{
			file >> gde.label;
			std::cout << "gde.label=" << gde.label << std::endl;
			have_object_label = true;
		}
		else if (tag.compare("grasp_type:") == 0)
		{
			file >> gde.grasp_type;
			std::cout << "gde.grasp_type=" << gde.grasp_type << std::endl;
			have_grasp_type = true;
		}
		else if (tag.compare("grasp_force:") == 0)
		{
			file >> gde.grasp_force;
			std::cout << "gde.grasp_force=" << gde.grasp_force << std::endl;
			have_grasp_force = true;
		}
		else if (tag.compare("grasp_offset_position:") == 0)
		{
			char c=' ';
			while (c!='[' && file.eof() == false)
				file.get(c);

			double val = 0.0;
			file >> val;
			t.setX(val);
			file >> val;
			t.setY(val);
			file >> val;
			t.setZ(val);
			std::cout << "gde.grasp_offset_position=[" << t.getX() << ", " << t.getY() << ", " << t.getZ() << "]" << std::endl;
			have_grasp_offset_position = true;
		}
		else if (tag.compare("grasp_offset_quaternion:") == 0)
		{
			char c=' ';
			while (c!='[' && file.eof() == false)
				file.get(c);

			double val = 0.0;
			file >> val;
			q.setW(val);
			file >> val;
			q.setX(val);
			file >> val;
			q.setY(val);
			file >> val;
			q.setZ(val);
			std::cout << "gde.grasp_offset_quaternion=[" << q.getW() << ", " << q.getX() << ", " << q.getY() << ", " << q.getZ() << "]" << std::endl;
			have_grasp_offset_quaternion = true;
		}
		file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

		// add complete data set to database
		if (have_object_label==true && have_grasp_type==true  && have_grasp_force==true && have_grasp_offset_position==true && have_grasp_offset_quaternion==true)
		{
			gde.grasp_offset = tf::Transform(q,t);
			grasp_database_[gde.label].push_back(gde);
			std::cout << "Added one database entry.\n\n";

			have_object_label = false;
			have_grasp_type = false;
			have_grasp_force = false;
			have_grasp_offset_position = false;
			have_grasp_offset_quaternion = false;
		}
	}

	std::cout << "Finished with reading the database.\n\n";

	file.close();
}


bool HermesGraspDatabase::computeGraspForDetection(hermes_grasp_database::GetGraspForDetection::Request &req, hermes_grasp_database::GetGraspForDetection::Response &res)
{
	// this callback function is executed each time a request comes in for this service server
	// here we just read the number from the request, square it and put the result into the response, the response is automatically sent back to the caller when this function returns

	ROS_INFO("GetGraspForDetection Service Server: Received a request for detected object %s at (x,y,z)=(%f, %f, %f), (wabc)=(%f, %f, %f, %f).",
			req.detection.label.c_str(), req.detection.pose.pose.position.x, req.detection.pose.pose.position.y, req.detection.pose.pose.position.z,
			req.detection.pose.pose.orientation.w, req.detection.pose.pose.orientation.x, req.detection.pose.pose.orientation.y, req.detection.pose.pose.orientation.z);

	// code for grasp database lookup
	res.grasp_configurations.clear();
	std::string object_label = req.detection.label;
	if (grasp_database_.find(object_label) != grasp_database_.end())
	{
		for (unsigned int i=0; i<grasp_database_[object_label].size(); i++)
		{
			tf::Transform offset = grasp_database_[object_label][i].grasp_offset;
			tf::Transform object_pose;
			tf::poseMsgToTF(req.detection.pose.pose, object_pose);

			// todo: transform into the desired coordinate system ?
			hermes_grasp_database::GraspConfiguration gc;
			tf::poseTFToMsg(offset*object_pose, gc.goal_position.pose);
			gc.goal_position.header = req.detection.pose.header;
			gc.grasp_type = grasp_database_[object_label][i].grasp_type;
			gc.grasp_force = grasp_database_[object_label][i].grasp_force;
			res.grasp_configurations.push_back(gc);
		}
	}

	// if the procedure fails, use "return false;" to inform the caller of the service
	if (res.grasp_configurations.size() == 0)
		return false;

	return true;	// tell the caller that the method was successfully executed
}
