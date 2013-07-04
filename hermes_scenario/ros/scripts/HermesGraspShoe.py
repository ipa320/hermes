#!/usr/bin/python
import roslib
roslib.load_manifest('hermes_scenario')
import rospy
import smach
import smach_ros

from HermesCommon import *
from HermesGenericGrasp import *

from cob_object_detection_msgs.msg import *
from cob_object_detection_msgs.srv import *
from hermes_grasp_database.msg import *
from hermes_grasp_database.srv import *



class DetectMarker(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['found', 'not_found', 'failed'],
			input_keys=['object_label'],
			output_keys=['detection'])

	def execute(self, userdata):
		sf = ScreenFormat("DetectMarker")
		print 'Searching for marker with label', userdata.object_label, '...'
		rospy.wait_for_service('/fiducials/get_fiducials',3.0)
		res = DetectObjectsResponse()
		detect_objects = rospy.ServiceProxy('/fiducials/get_fiducials', DetectObjects)
		try:
			req = DetectObjectsRequest()
			req.object_name.data = userdata.object_label
			res = detect_objects(req)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e, ' Object', userdata.object_label, 'was not found.'
			return 'not_found'

		for detection in res.object_list.detections:
			if detection.label == userdata.object_label:
				userdata.detection = detection
				#print detection

		return 'found'


class ComputeGrasp(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['found', 'not_found', 'failed'],
			input_keys=['detection'],
			output_keys=['grasp_configuration'])

	def execute(self, userdata):
		sf = ScreenFormat("ComputeGrasp")
		rospy.wait_for_service('/hermes_grasp_database/get_grasp_for_detection',3.0)
		res = GetGraspForDetectionResponse()
		get_grasp_for_detection = rospy.ServiceProxy('/hermes_grasp_database/get_grasp_for_detection', GetGraspForDetection)
		try:
			req = GetGraspForDetectionRequest()
			req.detection = userdata.detection
			res = get_grasp_for_detection(req)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e, ' No grasp for object', userdata.detection.label, 'available.'
			return 'not_found'

		userdata.grasp_configuration = res.grasp_configurations[0]
				
		return 'found'


class ShoePackaging(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
			outcomes=['finished', 'failed'],
			input_keys=['arm', 'hand', 'object_label'],
			output_keys=[])
		with self:

			smach.StateMachine.add('DETECT_SHOE', DetectMarker(),
									transitions={'found':'LOOKUP_GRASP',
											'not_found':'DETECT_SHOE',
											'failed':'failed'})

			smach.StateMachine.add('LOOKUP_GRASP', ComputeGrasp(),
									transitions={'found':'GRASP_SHOE',
												'not_found':'failed',
												'failed':'failed'})
			
			# grasping
			sm_generic_grasping = HermesGenericGrasping()
			smach.StateMachine.add('GRASP_SHOE', sm_generic_grasping,
                               transitions={'finished':'finished',
											'failed':'failed'},
                               remapping={'arm':'arm',
										  'hand':'hand',
										  'grasp_configuration':'grasp_configuration'})
			

if __name__ == '__main__':
	rospy.init_node("shoe_packaging")
	sm = ShoePackaging()
	
	# userdata
	sm.userdata.arm = 1;
	sm.userdata.hand = 1;
	sm.userdata.object_label='1'
	
	# introspection -> smach_viewer
	sis = smach_ros.IntrospectionServer('shoe_packaging_introspection', sm, '/SHOE_PACKAGING')
	sis.start()
	
	# start
	sm.execute()
	rospy.spin()
	sis.stop()