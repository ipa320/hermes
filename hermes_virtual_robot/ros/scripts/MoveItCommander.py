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
   
    # clean the scene
    scene.remove_world_object("grasp_point")
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
    pose.pose.position.x = 0.5636
    pose.pose.position.y = 0.6398
    pose.pose.position.z = 1.1455
    pose.pose.orientation.w = 0.15506
    pose.pose.orientation.x = 0.8216
    pose.pose.orientation.y = -0.32951
    pose.pose.orientation.z = 0.43857
    pose.header.frame_id = robot.get_planning_frame()
    pose.header.stamp = rospy.Time.now()

    
     
     
    
    group_left.set_pose_target(pose, "l_eef")
     
 
    result = group_left.go(None,1)
    
    if result == 0:
        print "ERROR init pos left"
        sys.exit()
    
    

    
    ####################### ADD RIGHT SHOE TO THE SCENE ###############################
    scene.remove_attached_object('r_eef', 'right_shoe')
    
    rospy.sleep(1)
    
    pose.pose.position.x = 0.475
    pose.pose.position.y = -0.1500
    pose.pose.position.z = 1.035
    pose.pose.orientation.w = 0.99144
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0.13053
    pose.header.frame_id = robot.get_planning_frame()
    pose.header.stamp = rospy.Time.now()
    path = roslib.packages.get_pkg_dir('hermes_virtual_robot')+'/common/files/stl/right_shoe.stl'
    scene.add_mesh("right_shoe", pose, path)
    rospy.sleep(1)
    ####################### PREGRASP RIGHT HAND ###############################
    req = rospy.ServiceProxy('grasp_hand', GraspHand)
    res = req(2,1) # 2 -> Right _hand 1-> PreGrasp
    res = req(1,1) # 2 -> Right _hand 1-> PreGrasp
    rospy.sleep(1)
    ####################### POSICION DE GRASP ###############################
    pose.pose.position.x = 0.4508
    pose.pose.position.y = -0.3605
    pose.pose.position.z = 1.1423
    pose.pose.orientation.w = 0.6812
    pose.pose.orientation.x = -0.21744
    pose.pose.orientation.y = 0.65471
    pose.pose.orientation.z = 0.24387
    pose.header.frame_id = robot.get_planning_frame()
    pose.header.stamp = rospy.Time.now()

     
     
    # move to a random target
    group_right.set_random_target()
    group_right.set_pose_target(pose, "r_eef")
     
     
    result = group_right.go(None,1)
    if result == 0:
        print "ERROR Grasp position"
        sys.exit()
    

    
    
    ####################### CLOSE RIGHT HAND ###############################
    scene.remove_world_object("right_shoe")
    rospy.sleep(1)
    res = req(2,2) # 2 -> Right _hand 2-> Grasp
    rospy.sleep(1)
    ####################### ATTACH RIGHT SHOE TO THE ROBOT ###############################
    pose.pose.position.x = 0.475
    pose.pose.position.y = -0.1500
    pose.pose.position.z = 1.035
    pose.pose.orientation.w = 0.99144
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0.13053
    
    scene.attach_mesh('r_eef', 'right_shoe', pose, roslib.packages.get_pkg_dir('hermes_virtual_robot')+'/common/files/stl/right_shoe.stl',links_off)
    rospy.sleep(1)
    
    
    ####################### POSICION ARRIBA ZAPATO ###############################
    pose.pose.position.x = 0.4493
    pose.pose.position.y = -0.3628
    pose.pose.position.z = 1.1937
    pose.pose.orientation.w = 0.68058
    pose.pose.orientation.x = -0.21466
    pose.pose.orientation.y = 0.65539
    pose.pose.orientation.z = 0.24739
    pose.header.frame_id = robot.get_planning_frame()
    pose.header.stamp = rospy.Time.now()
    
     
     
    # move to target
    group_right.set_pose_target(pose, "r_eef")
     
     
    result = group_right.go(None,1)
    
    if result == 0:
        print "ERROR Shoe UP"
        sys.exit()
        

    
    ####################### POSICION LANDSCAPE ###############################
    pose.pose.position.x = 0.5033
    pose.pose.position.y = -0.4402
    pose.pose.position.z = 1.3024
    pose.pose.orientation.w = 0.18857
    pose.pose.orientation.x = -0.13497
    pose.pose.orientation.y = 0.83443
    pose.pose.orientation.z = 0.49995
    pose.header.frame_id = robot.get_planning_frame()
    pose.header.stamp = rospy.Time.now()
    
     
     
    # move to target
    group_right.set_pose_target(pose, "r_eef")
     
     
    result = group_right.go(None,1)
    
    if result == 0:
        print "ERROR Landscape Position"
        sys.exit()
        
        
    ####################### CHANGE HAND RIGHT ###############################
    pose.pose.position.x = 0.5540
    pose.pose.position.y = -0.1764
    pose.pose.position.z = 1.2480
    pose.pose.orientation.w = 0.18836
    pose.pose.orientation.x = -0.1348
    pose.pose.orientation.y = 0.83323
    pose.pose.orientation.z = 0.50208
    pose.header.frame_id = robot.get_planning_frame()
    pose.header.stamp = rospy.Time.now()
    
     
     
    # move to target
    group_right.set_pose_target(pose, "r_eef")
     
     
    result = group_right.go(None,1)
    
    if result == 0:
        print "ERROR Change Hand Right"
        sys.exit()
        
    ####################### CHANGE HAND LEFT ###############################
    pose.pose.position.x = 0.5617
    pose.pose.position.y = 0.1244
    pose.pose.position.z = 1.4253
    pose.pose.orientation.w = 0.10902
    pose.pose.orientation.x = -0.13918
    pose.pose.orientation.y = -0.80607
    pose.pose.orientation.z = 0.56479
    pose.header.frame_id = robot.get_planning_frame()
    pose.header.stamp = rospy.Time.now()
    
     
     
    # move to target
    group_left.set_pose_target(pose, "l_eef")
     
     
    result = group_left.go(None,1)
    
    if result == 0:
        print "ERROR Change Hand Left"
        sys.exit()
        

    
    
  