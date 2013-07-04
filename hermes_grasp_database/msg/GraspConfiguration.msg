# message describing a grasp configuration

geometry_msgs/PoseStamped goal_position		# the target position of the wrist of the robot arm

int32 grasp_type				# one of the grasps defined in hermes_grasp_service/Grasp.srv

int32 grasp_force				# grasping force [0..100] (not applicable for OPENALL, STOPALL, FIRSTCALIBRATION, FASTCALIBRATION)