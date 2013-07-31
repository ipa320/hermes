#include "ros/ros.h"
#include "hermes_move_arm_action/HermesInterface.h"

// actions
#include <actionlib/server/simple_action_server.h>
#include <hermes_move_arm_action/MoveArmAction.h> // here you have to include the header file with exactly the same name as your message in the /action folder (the MessageAction.h is automatically generated from your Message.action file during compilation)

//tf ros package
#include "tf/tf.h"

//KDL
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>


// this typedef just establishes the abbreviation MoveArmActionServerType for the long data type
typedef actionlib::SimpleActionServer<hermes_move_arm_action::MoveArmAction> MoveArmActionServerType;

class HermesMoveArmActionServer
{
public:
	HermesMoveArmActionServer(ros::NodeHandle nh);
	void init();
	void moveArm(const hermes_move_arm_action::MoveArmGoalConstPtr& goal);


protected:
	HermesInterface hermesinterface;
	ros::NodeHandle node_;
	MoveArmActionServerType move_arm_action_server_; ///< Action server which accepts requests for moving the arm
};
