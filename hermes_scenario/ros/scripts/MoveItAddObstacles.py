#!/usr/bin/python
import roslib
roslib.load_manifest('hermes_scenario')
import rospy
import smach
import smach_ros
import actionlib
from geometry_msgs.msg import *

from moveit_commander import *


if __name__ == '__main__':
	rospy.init_node("hermes_scenario")
	psi = PlanningSceneInterface()
	rospy.sleep(20)
	pose = PoseStamped()
	pose.pose.position.x = 0.525
	pose.pose.position.y = 0.0
	pose.pose.position.z = 0.0
	pose.header.frame_id = "/pillar"
	pose.header.stamp = rospy.Time.now()
	path = roslib.packages.get_pkg_dir('hermes_scenario')+'/common/files/stl/big_table.stl'
	print path
	psi.add_mesh("table", pose, path)
	pose.pose.position.x = -1.4
	pose.pose.position.y = -0.26
	pose.pose.position.z = 0.0
	pose.pose.orientation.w = 0.7071
	pose.pose.orientation.x = 0
	pose.pose.orientation.y = 0
	pose.pose.orientation.z = -0.7071
	psi.add_mesh("table2", pose, path)
	#psi.remove_world_object("table")

	pose.pose.position.x = 0.425
	pose.pose.position.y = -0.2
	pose.pose.position.z = 0.78+0.125
	pose.pose.orientation.w = 1
	pose.pose.orientation.x = 0
	pose.pose.orientation.y = 0
	pose.pose.orientation.z = 0
	#psi.add_box("table_box", pose, size = (0.6, 0.8, 0.25))
	psi.add_box("table_box", pose, size = (1.2, 0.2, 0.75))

