#!/usr/bin/python
import rospy
import sys
import roslib

roslib.load_manifest('hermes_grasp_service')

from hermes_grasp_service.srv import *

# http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29



# LO MEJOR ES CREAR UN SERVICIO EN VIRTUAL ROBOT PARA CERRAR LA MANO


if __name__ == "__main__":
    
    print "Esperando al servicio ... grasp_service ... from hermes_grasp_service"
    rospy.wait_for_service('grasp_service')
    try:
        print "Service correct"
        req = rospy.ServiceProxy('grasp_service', HermesGrasp)
        # 1-> left Hand 2 -> Right Hand
        # 20-> pregrasp; 11-> grasp; 7-> open
        res = req(2,7,100)
        print "Message: %s"%res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
  