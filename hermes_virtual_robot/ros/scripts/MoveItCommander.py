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
    pose = PoseStamped()
    rospy.sleep(1)
   
    # clean the scene
    scene.remove_world_object("big_table")
    scene.remove_world_object("big_table_2")
    scene.remove_world_object("small_table")
    scene.remove_world_object("right_shoe")
    scene.remove_world_object("grasp_point")
    
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
   
    #Get Current state
    state = robot.get_current_variable_values()
    
    #Set scene objects
   
    pose.pose.position.x = 0.525
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.0
    pose.header.frame_id = robot.get_planning_frame()
    pose.header.stamp = rospy.Time.now()
    path = roslib.packages.get_pkg_dir('hermes_virtual_robot')+'/common/files/stl/big_table.stl'
    scene.add_mesh("big_table", pose, path)

    pose.pose.position.x = -1.4
    pose.pose.position.y = -0.26
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 0.7071
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = -0.7071
    pose.header.frame_id = robot.get_planning_frame()
    pose.header.stamp = rospy.Time.now()
    path = roslib.packages.get_pkg_dir('hermes_virtual_robot')+'/common/files/stl/big_table.stl'
    scene.add_mesh("big_table_2", pose, path)
    
    pose.pose.position.x = 0.425
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.78+0.125
    pose.pose.orientation.w = 1
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.header.frame_id = robot.get_planning_frame()
    pose.header.stamp = rospy.Time.now()
    scene.add_box("small_table", pose, size = (0.6, 0.8, 0.255))
   

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
    
    # pick an object
#     robot.r_arm.pick("grasp_point")
# 
#     rospy.spin()
#     roscpp_shutdown()
    
    
#     ##########  GO TO POS INIT ######################
# 
#     print "Current state:"
#     state = robot.get_current_variable_values()
#     print state
#     
#     print "state['r_joint1']: ", state['r_joint1'];
#     
#     # plan to a random location 
#     a = robot.r_arm
#     a.set_start_state(RobotState())
#     r = a.get_random_joint_values()
#     print "Planning to random joint position: "
#     print r
#     p = a.plan(r)
#     print "Solution:"
#     print p
# 
#     roscpp_shutdown()


    ####################### POSICION DE GRASP ###############################
    pose.pose.position.x = 0.52
    pose.pose.position.y = -0.3463
    pose.pose.position.z = 1.1356
    pose.pose.orientation.w = 0.64445
    pose.pose.orientation.x = -0.23867
    pose.pose.orientation.y = 0.68061
    pose.pose.orientation.z = 0.25395
    pose.header.frame_id = robot.get_planning_frame()
    pose.header.stamp = rospy.Time.now()

    group_right = MoveGroupCommander("r_arm")
    group_left = MoveGroupCommander("l_arm")
     
     
    # move to a random target
    group_right.set_random_target()
    group_right.set_pose_target(pose, "r_eef")
     
     
    group_right.go()
    
    rospy.sleep(10)
    ####################### POSICION ARRIBA ZAPATO ###############################
    pose.pose.position.x = 0.52
    pose.pose.position.y = -0.3463
    pose.pose.position.z = 1.256
    pose.pose.orientation.w = 0.64445
    pose.pose.orientation.x = -0.23867
    pose.pose.orientation.y = 0.68061
    pose.pose.orientation.z = 0.25395
    pose.header.frame_id = robot.get_planning_frame()
    pose.header.stamp = rospy.Time.now()

     
     
    # move to target
    group_right.set_pose_target(pose, "r_eef")
     
     
    group_right.go()
    
    
    ##### IMPRIMIENDO LA POSICION FINAL
    rospy.sleep(10)
    
    
    state = robot.get_current_variable_values()
    print "---------------------------------------"
    print "FINAL STATE RIGHT ARM"
    print "---------------------------------------"
    print "state['r_joint1']: ", state['r_joint1'];
    print "state['r_joint2']: ", state['r_joint2'];
    print "state['r_joint3']: ", state['r_joint3'];
    print "state['r_joint4']: ", state['r_joint4'];
    print "state['r_joint5']: ", state['r_joint5'];
    print "state['r_joint6']: ", state['r_joint6'];
    print "state['r_joint7']: ", state['r_joint7'];
