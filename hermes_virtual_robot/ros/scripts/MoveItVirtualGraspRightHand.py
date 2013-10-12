#!/usr/bin/python
import rospy
import sys
import roslib

roslib.load_manifest('hermes_virtual_robot')

from hermes_virtual_robot.srv import *

# http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29



# LO MEJOR ES CREAR UN SERVICIO EN VIRTUAL ROBOT PARA CERRAR LA MANO


if __name__ == "__main__":
    
    print "Esperando al servicio ... grasp_hand"
    rospy.wait_for_service('grasp_hand')
    try:
        print "Service correct"
        req = rospy.ServiceProxy('grasp_hand', GraspHand)
        # 1-> left Hand 2 -> Right Hand
        # 1-> pregrasp; 2-> grasp; 3-> open
        res = req(2,2)
        print "Message: %s"%res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
  