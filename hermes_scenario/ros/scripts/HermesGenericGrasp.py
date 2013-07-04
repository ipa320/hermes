#!/usr/bin/python
import roslib
roslib.load_manifest('hermes_scenario')
import rospy
import smach
import smach_ros
import actionlib

from HermesCommon import *

from hermes_grasp_database.msg import *
from hermes_grasp_database.srv import *
from hermes_grasp_service.srv import *
from geometry_msgs.msg import *
from hermes_move_arm_action.msg import *


def Grasp(hand, grasp_type, grasp_force):
		print 'Executing grasp number', grasp_type, 'for hand', hand, 'with grasp_force', grasp_force, '...'
		rospy.wait_for_service('/hermes_grasp_service/grasp_service',3.0)
		res = HermesGraspResponse()
 		grasp = rospy.ServiceProxy('/hermes_grasp_service/grasp_service', HermesGrasp)
 		try:
 			req = HermesGraspRequest()
 			req.hand = hand
 			req.grasp_type = grasp_type
 			req.grasp_force = grasp_force
 			res = grasp(req)
 		except rospy.ServiceException, e:
 			print "Service call failed: %s"%e
 			return 'failed'

		return 'success'
	
	
class OpenHand(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['success', 'failed'],
			input_keys=['hand'],
			output_keys=[])

	def execute(self, userdata):
		sf = ScreenFormat("OpenHand")
		grasp_type = 7
		grasp_force = 100
		
		return Grasp(userdata.hand, grasp_type, grasp_force)


class GraspObject(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['success', 'failed'],
			input_keys=['hand', 'grasp_configuration'],
			output_keys=[])

	def execute(self, userdata):
		sf = ScreenFormat("GraspObject")		
		return Grasp(userdata.hand, userdata.grasp_configuration.grasp_type, userdata.grasp_configuration.grasp_force)


class MoveArm(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['success', 'failed'],
			input_keys=['arm', 'grasp_configuration'],
			output_keys=[])
		self.client = actionlib.SimpleActionClient('/hermes_move_arm_action/move_arm_action', MoveArmAction)

	def execute(self, userdata):
		sf = ScreenFormat("MoveArm")
		print 'Moving arm', userdata.arm, 'to position', userdata.grasp_configuration.goal_position, '...'
		
		if not self.client.wait_for_server(rospy.Duration.from_sec(3.0)):
			print '/hermes_move_arm_action/move_arm_action action server not available'
			return 'failed'
		
		goal = MoveArmGoal()
		goal.arm = userdata.arm
		goal.goal_position = userdata.grasp_configuration.goal_position

		self.client.send_goal(goal)
		if not self.client.wait_for_result():#rospy.Duration.from_sec(5.0)):
			return 'failed'
		
		return 'success'


class HermesGenericGrasp(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
			outcomes=['finished', 'failed'],
			input_keys=['arm', 'hand', 'grasp_configuration'],
			output_keys=[])
		with self:

			smach.StateMachine.add('OPEN_HAND', OpenHand(),
								transitions={'success':'MOVE_ARM_TO_OBJECT',
										'failed':'failed'})

			smach.StateMachine.add('MOVE_ARM_TO_OBJECT', MoveArm(),
									transitions={'success':'GRASP_OBJECT',
											'failed':'failed'})

			smach.StateMachine.add('GRASP_OBJECT', GraspObject(),
								transitions={'success':'finished',
										'failed':'failed'})

if __name__ == '__main__':
	rospy.init_node("hermes_generic_grasp")
	sm = HermesGenericGrasp()
	
	# userdata
	sm.userdata.hand = 1
	sm.userdata.arm = 1
	sm.userdata.grasp_configuration = GraspConfiguration()
	sm.userdata.grasp_configuration.goal_position = PoseStamped()
	sm.userdata.grasp_configuration.grasp_type = 12
	sm.userdata.grasp_configuration.grasp_force = 80
	
	# introspection -> smach_viewer
	sis = smach_ros.IntrospectionServer('hermes_generic_grasp_introspection', sm, '/HERMES_GENERIC_GRASP')
	sis.start()
	
	# start
	sm.execute()
	rospy.spin()
	sis.stop()