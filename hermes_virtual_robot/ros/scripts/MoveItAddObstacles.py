#!/usr/bin/python
import roslib
roslib.load_manifest('hermes_virtual_robot')
import rospy
import smach
import smach_ros
import actionlib
from geometry_msgs.msg import *

from moveit_commander import *


if __name__ == '__main__':
	rospy.init_node("hermes_virtual_addObjects")
	robot = RobotCommander()
	scene = PlanningSceneInterface()
	rospy.sleep(2)
	pose = PoseStamped()
	
	# clean the scene
	scene.remove_world_object("big_table")
	scene.remove_world_object("big_table_2")
	scene.remove_world_object("small_table")
	scene.remove_world_object("right_shoe")
	scene.remove_world_object("left_shoe")
	scene.remove_world_object("shoe_box")

	
	#Set scene objects
	
	# Big Table
	pose.pose.position.x = 0.525
	pose.pose.position.y = 0.0
	pose.pose.position.z = 0.0
	pose.header.frame_id = robot.get_planning_frame()
	pose.header.stamp = rospy.Time.now()
	path = roslib.packages.get_pkg_dir('hermes_virtual_robot')+'/common/files/stl/big_table.stl'
	scene.add_mesh("big_table", pose, path)
	
	# Big Table 2
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
	
	# Small Table
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
	
	# Right Shoe
# 	pose.pose.position.x = 0.475
# 	pose.pose.position.y = -0.1500
# 	pose.pose.position.z = 1.035
# 	pose.pose.orientation.w = 0.99144
# 	pose.pose.orientation.x = 0
# 	pose.pose.orientation.y = 0
# 	pose.pose.orientation.z = 0.13053
# 	pose.header.frame_id = robot.get_planning_frame()
# 	pose.header.stamp = rospy.Time.now()
# 	path = roslib.packages.get_pkg_dir('hermes_virtual_robot')+'/common/files/stl/right_shoe.stl'
# 	scene.add_mesh("right_shoe", pose, path)
	
	# Left Shoe
	pose.pose.position.x = 0.275
	pose.pose.position.y = -0.1500
	pose.pose.position.z = 1.035
	pose.pose.orientation.w = 0.99144
	pose.pose.orientation.x = 0
	pose.pose.orientation.y = 0
	pose.pose.orientation.z = 0.13053
	pose.header.frame_id = robot.get_planning_frame()
	pose.header.stamp = rospy.Time.now()
	path = roslib.packages.get_pkg_dir('hermes_virtual_robot')+'/common/files/stl/left_shoe.stl'
	scene.add_mesh("left_shoe", pose, path)
	
	# Shoe Box
	pose.pose.position.x = 0.40
	pose.pose.position.y = 0.15
	pose.pose.position.z = 1.035
	pose.pose.orientation.w = 0.70711
	pose.pose.orientation.x = 0
	pose.pose.orientation.y = 0
	pose.pose.orientation.z = 0.70711
	pose.header.frame_id = robot.get_planning_frame()
	pose.header.stamp = rospy.Time.now()
	path = roslib.packages.get_pkg_dir('hermes_virtual_robot')+'/common/files/stl/shoe_box_17_29.stl'
	scene.add_mesh("shoe_box", pose, path)
