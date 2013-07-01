#include "ros/ros.h"

// actions
#include <actionlib/client/simple_action_client.h>
#include <hermes_move_arm_action/MoveArmAction.h> // here you have to include the header file with exactly the same name as your message in the /action folder (the MessageAction.h is automatically generated from your Message.action file during compilation)

// this typedef just establishes the abbreviation SquareActionServer for the long data type
typedef actionlib::SimpleActionClient<hermes_move_arm_action::MoveArmAction> HermesMoveArmClientType;

class HermesMoveArmActionClient
{
public:
	HermesMoveArmActionClient(ros::NodeHandle nh);
	bool init();
	void run();

protected:
	void doneCb(const actionlib::SimpleClientGoalState& state, const hermes_move_arm_action::MoveArmResultConstPtr& result);
	void activeCb();
	void feedbackCb(const hermes_move_arm_action::MoveArmFeedbackConstPtr& feedback);

	ros::NodeHandle node_;
	HermesMoveArmClientType move_arm_action_client_; ///< Action server which accepts requests for moving the arm
};
