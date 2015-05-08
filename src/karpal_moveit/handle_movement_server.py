#! /usr/bin/env python

from karpal_moveit.srv import *
import rospy
from karpal_moveit import *

def handle_movement(req):
	print "Returning [%s, %s, %s]"%(req.x, req.x, req.z)
	testLeftArm()
	return HandleMovementResponse("Success")

def handle_movement_server():
	rospy.init_node('handle_movement_server')
	s = rospy.Service('handle_movement', HandleMovement, handle_movement)
	print "Server started"
	rospy.spin()

if __name__ == "__main__":
	handle_movement_server()