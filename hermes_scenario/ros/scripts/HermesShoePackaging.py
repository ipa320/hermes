#!/usr/bin/python
import roslib
roslib.load_manifest('hermes_scenario')
import rospy
import smach
import smach_ros

from HermesCommon import *
from HermesGenericGrasp import *
from HermesGraspShoe import *


class HermesShoePackaging(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
			outcomes=['finished', 'failed'],
			input_keys=[],
			output_keys=[])
		with self:

			self.userdata.arm = 1;		# 1=left. 2=right
			self.userdata.hand = 1;
			self.userdata.object_label='1'
			
			# grasping
			sm_grasp_shoe = HermesGraspShoe()
			smach.StateMachine.add('GRASP_SHOE', sm_grasp_shoe,
                               transitions={'finished':'finished',
											'failed':'failed'},
                               remapping={'arm':'arm',
										  'hand':'hand',
										  'object_label':'object_label'})
			

if __name__ == '__main__':
	rospy.init_node("hermes_shoe_packaging")
	sm = HermesShoePackaging()
	
	# userdata
# 	sm.userdata.arm = 1;
# 	sm.userdata.hand = 1;
# 	sm.userdata.object_label='1'
	
	# introspection -> smach_viewer
	sis = smach_ros.IntrospectionServer('hermes_shoe_packaging_introspection', sm, '/HERMES_SHOE_PACKAGING')
	sis.start()
	
	# start
	sm.execute()
	rospy.spin()
	sis.stop()