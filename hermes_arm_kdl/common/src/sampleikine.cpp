#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
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
  KDL::ChainIkSolverVel_pinv iksolver1v(hermes_chain);

  // Create solver based on kinematic chain
  KDL::ChainIkSolverPos_NR iksolver = ChainIkSolverPos_NR(hermes_chain,fksolver,iksolver1v,100,1e-3);




 // Create the vector  that will contain the results
    KDL::Vector vectorPos(0.45,0.101,-1.144);  

 // Create the rotation  that will contain the results
    KDL::Rotation rotationPos(0.70,-0.43,0.55,-0.16,-0.86,-0.46,0.68,0.23,-0.688);  

 // Create the frame that will contain the results
   KDL::Frame cartpos(rotationPos,vectorPos);  


  // Create joint array
  KDL::JntArray q(hermes_chain.getNrOfJoints());

// Create joint array
  KDL::JntArray q_init(hermes_chain.getNrOfJoints());
  
 // Assign values to the joints
 q_init(0) = 0.11;
 q_init(1) = -0.62;
 q_init(2) = 0.36;
 q_init(3) = -0.12;
 q_init(4) = 1.1;
 q_init(5) = -0.35;
 q_init(6) = 0.75;

 q(0) = 0.23;
 q(1) = -0.32;
 q(2) = 0.3;
 q(3) = 0.2;
 q(4) = 0.15;
 q(5) = 0.20;
 q(6) = 0.14;
   

for(int i=0;i<7;i++)
	      std::cout << q(i) <<std::endl;
  // Calculate forward position kinematics
  bool kinematics_status;
  kinematics_status = fksolver.JntToCart(q,cartpos);
  std::cout << cartpos <<std::endl;
  kinematics_status = iksolver.CartToJnt(q_init,cartpos,q);
  if(kinematics_status>=0){
	for(int i=0;i<7;i++)
	      std::cout << q(i) <<std::endl;
      printf("%s \n","Succes, thanks KDL!");
  }else{
      printf("%s \n","Error: could not calculate forward kinematics :(");
  }
   kinematics_status = fksolver.JntToCart(q,cartpos);
  std::cout << cartpos <<std::endl;

  return 0;
}
