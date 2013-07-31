#include "hermes_arm_kdl/hermes_arm_kdl_service_server.h"
#include "ros/package.h"


HermesArmKdlServiceServer::HermesArmKdlServiceServer(ros::NodeHandle nh)
{
	node_ = nh;
}


void HermesArmKdlServiceServer::init()
{
	/**
	* The advertiseService() function is how you tell ROS that you want to provide a service for other modules (software nodes).
	*/

	std::string urdf_file = ros::package::getPath("hermes_arm_kdl") + "/urdf/hermesArm.urdf";
	hermes_kdl.init(urdf_file);

	hermes_arm_kdl_service_fkine_server_ = node_.advertiseService("arm_kdl_service_fkine_server", &HermesArmKdlServiceServer::fkine, this);
	hermes_arm_kdl_service_ikine_server_ = node_.advertiseService("arm_kdl_service_ikine_server", &HermesArmKdlServiceServer::ikine, this);


}


bool HermesArmKdlServiceServer::fkine(hermes_arm_kdl::Fkine::Request &req, hermes_arm_kdl::Fkine::Response &res)
{

	
	KDL::JntArray q = KDL::JntArray(7);
	KDL::Frame cartPos;


	//Convert data from request to KDL
	for (int i=0;i<7;i++)
		q(i)=req.jointAngles[i];


	hermes_kdl.computeFkine(q, cartPos);



	// Convert data for return
	for (int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			res.rotation[3*i+j] = cartPos(i, j);


	for (int i=0;i<3;i++)
		res.position[i] = cartPos(i, 3);


	// if the procedure fails, use "return false;" to inform the caller of the service
	res.message = "success";
	//res.fkine = req.test+1;
	return true;	// tell the caller that the method was successfully executed
}

bool HermesArmKdlServiceServer::ikine(hermes_arm_kdl::ikine::Request &req, hermes_arm_kdl::ikine::Response &res)
{


	KDL::JntArray q = KDL::JntArray(7);
	KDL::JntArray q_init = KDL::JntArray(7);

	KDL::Vector pos(req.position[0],req.position[1],req.position[2]);
	KDL::Rotation rot(req.rotation[0],req.rotation[1],req.rotation[2],req.rotation[3],req.rotation[4],req.rotation[5],req.rotation[6],req.rotation[7],req.rotation[8]);
	KDL::Frame cartPos(rot,pos);

	for (int i=0;i<7;i++)
			q_init(i)=req.jointAngles_init[i];

	hermes_kdl.computeikine( q_init, cartPos, q);

	std::cout << "cartPos: " << std::endl;
	std::cout << cartPos <<std::endl;



	for (int i=0;i<7;i++)
	{
			while(q(i) > KDL::PI){
				q(i) -= 2*KDL::PI;
			}

			while(q(i) < -KDL::PI){
				q(i) += 2*KDL::PI;
			}


	}

	for (int i=0;i<7;i++){
		res.jointAngles[i] = q(i);
		std::cout << "q_" << i << " : " << q(i) <<std::endl;
	}




	// if the procedure fails, use "return false;" to inform the caller of the service
	res.message = "success";
	//res.fkine = req.test+1;
	return true;	// tell the caller that the method was successfully executed
}


