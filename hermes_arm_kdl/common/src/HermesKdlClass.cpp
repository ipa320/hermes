#include "hermes_arm_kdl/HermesKdlClass.h"

HermesKdlClass::HermesKdlClass()
{
	fksolver = 0;
	iksolver1v = 0;
	iksolver = 0;


}

HermesKdlClass::~HermesKdlClass()
{
	if (fksolver != 0)
		delete fksolver;
	if (iksolver1v != 0)
			delete iksolver1v;
	if (iksolver != 0)
			delete iksolver;

}

int HermesKdlClass::init(const std::string& urdf_file)
{
	KDL::Tree hermes_tree;
	urdf::Model hermes_model;
	if (!hermes_model.initFile(urdf_file)) {
		std::cout << "Failed to parse urdf file" << std::endl;
		return -1;
	}
	if (!kdl_parser::treeFromUrdfModel(hermes_model, hermes_tree)) {
		std::cout << "Failed to construct kdl tree" << std::endl;
		return -1;
	}
	std::cout  << "Successfully parsed urdf file to KDL" << std::endl;


	// Get Chain from the Tree
	bool exit_value;
	exit_value = hermes_tree.getChain("base", "link7", hermes_chain);

	// Create solver based on kinematic chain

	fksolver = new KDL::ChainFkSolverPos_recursive(hermes_chain);

	// Create joint array
	q(hermes_chain.getNrOfJoints());

	// Create iKineSolverVel_pinv
	iksolver1v = new KDL::ChainIkSolverVel_pinv(hermes_chain);

	// ChainIkSolverPos_NR
	iksolver = new KDL::ChainIkSolverPos_NR(hermes_chain,*fksolver,*iksolver1v,100,1e-3);

	return 0;
}

void HermesKdlClass::computeFkine(KDL::JntArray q, KDL::Frame &cartPos)
{
	bool kinematics_status;
	  kinematics_status = fksolver->JntToCart(q,cartPos);


}

void HermesKdlClass::computeikine( KDL::JntArray  q_init, KDL::Frame cartPos, KDL::JntArray  &q)
{
	bool kinematics_status;
		kinematics_status = iksolver->CartToJnt(q_init,cartPos,q);

}

