#!/usr/bin/python
import rospy
import sys
import roslib
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, roscpp_shutdown, PlanningSceneInterface
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

roslib.load_manifest('hermes_virtual_robot')

from hermes_virtual_robot.srv import *


#http://moveit.ros.org/doxygen/annotated.html
#https://github.com/ros-planning/moveit_commander

if __name__ == '__main__':
    
  
    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    rospy.wait_for_service('grasp_hand')
    
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    pose = PoseStamped()
    
    rospy.sleep(1)
   
    
    
   
    groups=[]
    groups.append("r_arm")
    groups.append("l_arm")
   
    # move_group
    move_group_right = MoveGroupCommander("r_arm")
    move_group_left = MoveGroupCommander("l_arm")
 
   
    #Get Current state
    state = robot.get_current_variable_values()
    
    


    ####################### INIT POSITION RIGHT ARM ###############################
    move_group_right.set_random_target()
    result = move_group_right.go(None,0)
     
 
        
    ####################### INIT POSITION LEFT ARM ###############################    
    move_group_left.set_random_target()
     
 
    result = move_group_left.go(None,0)
    
    if result == 0:
        print "ERROR Moving"
        sys.exit()
    
    

    
    

    
    
  