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

roslib.load_manifest('hermes_grasp_service')

from hermes_grasp_service.srv import *


#http://moveit.ros.org/doxygen/annotated.html
#https://github.com/ros-planning/moveit_commander

if __name__ == '__main__':
    
  
    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    print "Esperando al servicio ... grasp_service ... from hermes_grasp_service"
    rospy.wait_for_service('grasp_service')
    
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    pose = PoseStamped()
    
    rospy.sleep(1)
   
    # clean the scene
    scene.remove_world_object("right_shoe")
    
    # move_group
    group_right = MoveGroupCommander("r_arm")
    group_left = MoveGroupCommander("l_arm")
   
   
    #Get Current state
    state = robot.get_current_variable_values()
    
    # Link off contact
    links_off=[]
    links_off.append("r_thumb1")
    links_off.append("r_thumb2")
    links_off.append("r_thumb3")
    links_off.append("r_index1")
    links_off.append("r_index2")
    links_off.append("r_middle1")
    links_off.append("r_middle2")
    links_off.append("r_ring1")
    links_off.append("r_ring2")
    links_off.append("r_pinky1")
    links_off.append("r_pinky2")
    
    ####################### DEATTACH SHOE  ###############################
    scene.remove_attached_object('r_eef', 'right_shoe')    
    
    ####################### OPEN RIGHT HAND  ###############################    
    try:
        print "Service correct"
        req = rospy.ServiceProxy('grasp_service', HermesGrasp)
        # 1-> left Hand 2 -> Right Hand
        # 1-> pregrasp; 2-> grasp; 3-> open
        res = req(2,7,100)
        print "Message: %s"%res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e    
        
    ####################### OPEN LEFT HAND  ###############################    
    try:
        print "Service correct"
        req = rospy.ServiceProxy('grasp_service', HermesGrasp)
        # 1-> left Hand 2 -> Right Hand
        # 1-> pregrasp; 2-> grasp; 3-> open
        res = req(1,7,100)
        print "Message: %s"%res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


    ####################### INIT POSITION RIGHT ARM ###############################
    pose.pose.position.x = 0.5410
    pose.pose.position.y = -0.6719
    pose.pose.position.z = 1.3042
    pose.pose.orientation.w = 0.68025
    pose.pose.orientation.x = -0.2169
    pose.pose.orientation.y = 0.65613
    pose.pose.orientation.z = 0.24435
    pose.header.frame_id = robot.get_planning_frame()
    pose.header.stamp = rospy.Time.now()

    
     
     
   
    group_right.set_pose_target(pose, "r_eef")
     
 
    result = group_right.go(None,1)
    
    if result == 0:
        print "ERROR init pos right"
        sys.exit()
        
        
    ####################### INIT POSITION LEFT ARM ###############################    
    pose.pose.position.x = 0.4895
    pose.pose.position.y = 0.5832
    pose.pose.position.z = 1.4003
    pose.pose.orientation.w = 0.27017
    pose.pose.orientation.x = 0.61248
    pose.pose.orientation.y = -0.33713
    pose.pose.orientation.z = 0.66198
    pose.header.frame_id = robot.get_planning_frame()
    pose.header.stamp = rospy.Time.now()

    
     
     
    
    group_left.set_pose_target(pose, "l_eef")
     
 
    result = group_left.go(None,1)
    
    if result == 0:
        print "ERROR init pos left"
        sys.exit()
        
    ####################### PULGAR DERECHO PREGRASP  ###############################    
    try:
        print "Service correct"
        req = rospy.ServiceProxy('grasp_service', HermesGrasp)
        # 1-> left Hand 2 -> Right Hand
        # 1-> pregrasp; 2-> grasp; 3-> open
        res = req(2,20,100)
        print "Message: %s"%res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e    
        
    ####################### PULGAR IZQUIERDO PREGRASP  ###############################    
    try:
        print "Service correct"
        req = rospy.ServiceProxy('grasp_service', HermesGrasp)
        # 1-> left Hand 2 -> Right Hand
        # 1-> pregrasp; 2-> grasp; 3-> open
        res = req(1,20,100)
        print "Message: %s"%res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
   
    
    
  