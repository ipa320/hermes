#!/usr/bin/python
import rospy
import sys
import roslib
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, roscpp_shutdown, PlanningSceneInterface
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


#http://moveit.ros.org/doxygen/annotated.html
#https://github.com/ros-planning/moveit_commander

if __name__ == '__main__':
    
  
    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    rospy.sleep(1)
   

    
    #Print Current State
    state = robot.get_current_variable_values()
    
    print "---------------------------------------"
    print "CURRENT STATE RIGHT ARM"
    print "---------------------------------------"
    print "state['r_joint1']: ", state['r_joint1'];
    print "state['r_joint2']: ", state['r_joint2'];
    print "state['r_joint3']: ", state['r_joint3'];
    print "state['r_joint4']: ", state['r_joint4'];
    print "state['r_joint5']: ", state['r_joint5'];
    print "state['r_joint6']: ", state['r_joint6'];
    print "state['r_joint7']: ", state['r_joint7'];
   
   