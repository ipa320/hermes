#include "ros/ros.h"
#include "hermes_reference_frames_service/hermes_reference_frames_service_server.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "reference_frames_service");
  ros::NodeHandle n;

  HermesReferenceFramesServiceServer srv(n);

  // initialize the service server (for details see comments in class)
  srv.init();
  ros::spin();

  return 0;
}
