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


class HermesKdlClass
{
private:
	KDL::Chain hermes_chain;
	KDL::ChainFkSolverPos_recursive* fksolver;
	KDL::JntArray q;
	KDL::ChainIkSolverVel_pinv* iksolver1v;
	KDL::ChainIkSolverPos_NR* iksolver;

public:
	HermesKdlClass();
	~HermesKdlClass();
	int init(const std::string& urdf_file);

	void computeFkine(KDL::JntArray q, KDL::Frame &cartPos);
	void computeikine( KDL::JntArray  q_init, KDL::Frame cartPos, KDL::JntArray  &q);



};
