#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kdl.hpp>
#include <urdf/model.h>
#include "ros/ros.h"

using namespace KDL;

int main(int argc, char** argv){
  ros::init(argc, argv, "my_parser");
  if (argc != 2){
    ROS_ERROR("Need a urdf file as argument");
    return -1;
  }
  std::string urdf_file = argv[1];

  KDL::Tree hermes_tree;
  urdf::Model hermes_model;
  if (!hermes_model.initFile(urdf_file)){
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
  if (!kdl_parser::treeFromUrdfModel(hermes_model,hermes_tree)){
	ROS_ERROR("Failed to construct kdl tree");
	return -1;
  }
  ROS_INFO("Successfully parsed urdf file to KDL");
 
  ROS_INFO("Try to solve forward kinematics for zero joint position ...");

  // Get Chain from the Tree
  bool exit_value;
  KDL::Chain hermes_chain;
  exit_value = hermes_tree.getChain("base","link7",hermes_chain);


  // Create solver based on kinematic chain
  KDL::ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(hermes_chain);

  // Create joint array
  KDL::JntArray q(hermes_chain.getNrOfJoints());
  
  // Assign values to the joints
  /*for(unsigned int i=0;i<hermes_chain.getNrOfJoints();i++){
        q(i)=0.0;
    }*/
 q(0) = -0.1;
 q(1) = 1.2;
 q(2) = -0.3;
 q(3) = 0.6;
 q(4) = 0.4;
 q(5) = 0.75;
 q(6) = -0.1;

  // Create the frame that will contain the results
    KDL::Frame cartpos;    
 
  // Calculate forward position kinematics
  bool kinematics_status;
  kinematics_status = fksolver.JntToCart(q,cartpos);
  if(kinematics_status>=0){
      std::cout << cartpos <<std::endl;
      printf("%s \n","Succes, thanks KDL!");
  }else{
      printf("%s \n","Error: could not calculate forward kinematics :(");
  }


  return 0;
}
