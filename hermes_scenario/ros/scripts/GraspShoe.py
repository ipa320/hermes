#!/usr/bin/python
import roslib
roslib.load_manifest('hermes_scenario')
import rospy
import smach
import smach_ros

from cob_object_detection_msgs.msg import *
from cob_object_detection_msgs.srv import *
#from sensor_msgs.msg import RegionOfInterest

'''
class DetectMarker(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['success', 'failed'],
			input_keys=['detection'],
			output_keys=['detection'])
		/fiducials/get_fiducials
		#self.client = actionlib.SimpleActionClient('/trigger_segmentation', TriggerAction)

	def execute(self, userdata):
		#stop mapping
		# goal = TriggerGoal()
		# goal.start = False
		# if not self.client.wait_for_server(rospy.Duration.from_sec(3.0)):#rospy.Duration.from_sec(5.0)):
		# rospy.logerr('Trigger action server not available')
		# return 'failed'

		rospy.wait_for_service('geometry_map/clear_map',2.0)
		try:
			clear_geom_map = rospy.ServiceProxy('geometry_map/clear_map', Trigger)
			resp1 = clear_geom_map()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

		#start mapping
		# goal.start = True
		# if not self.client.wait_for_server(rospy.Duration.from_sec(1.0)):
		# rospy.logerr('Trigger action server not available')
		# return 'failed'
		# self.client.send_goal(goal)
		# if not self.client.wait_for_result():
		# return 'failed'


		#stop mapping
		# goal = TriggerGoal()
		# goal.start = False
		# if not self.client.wait_for_server(rospy.Duration.from_sec(1.0)):#rospy.Duration.from_sec(5.0)):
		# rospy.logerr('Trigger action server not available')
		# return 'failed'
		# self.client.send_goal(goal)
		# if not self.client.wait_for_result():#rospy.Duration.from_sec(5.0)):
		# return 'failed'

		#trigger table extraction
		rospy.wait_for_service('table_extraction/get_tables',3.0)
		try:
			extract_tables = rospy.ServiceProxy('table_extraction/get_tables', GetTables)
			userdata.tables = extract_tables().tables
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		print "Found %d tables"%len(userdata.tables.shapes)
		if len(userdata.tables.shapes) == 0:
			return 'not_found'
		return 'found'
'''

class DetectMarker(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['success', 'failed'],
			input_keys=['detection'],
			output_keys=['detection'])

	def execute(self, userdata):
		rospy.wait_for_service('/fiducials/get_fiducials',3.0)
		detections=[]
		detect_objects = rospy.ServiceProxy('/fiducials/get_fiducials', DetectObjects)
		try:
			req = DetectObjectsRequest()
			req.object_name.data = 'all'
			detections = detect_objects(req).object_list
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

		print detections

		return 'success'


class ShoePackaging(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
			outcomes=['finished', 'failed'])
		with self:

			smach.StateMachine.add('DETECT_SHOE', DetectMarker(),
									transitions={'success':'finished',
											'failed':'failed'})
	'''
			smach.StateMachine.add('COMPUTE_GRASP', ComputeGrasp(),
									transitions={'success':'OPEN_HAND',
											'failed':'failed'})

			smach.StateMachine.add('OPEN_HAND', Grasp(),
								transitions={'success':'MOVE_ARM_TO_SHOE',
										'failed':'failed'})

			smach.StateMachine.add('MOVE_ARM_TO_SHOE', MoveArm(),
									transitions={'success':'GRASP_SHOE',
											'failed':'failed'})

			smach.StateMachine.add('GRASP_SHOE', Grasp(),
								transitions={'success':'finished',
										'failed':'failed'})
	'''

if __name__ == '__main__':
	rospy.init_node("shoe_packaging")
	SM = ShoePackaging()
	SM.execute()
	rospy.spin()
