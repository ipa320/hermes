
#include "hermes_traj_action/traj_action_server.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "traj_action_server");

  ros::NodeHandle node;//("~");
  TrajActionServer jte(node);

   ros::spin();
    return 0;
}



