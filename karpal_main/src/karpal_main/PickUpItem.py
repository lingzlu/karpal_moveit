#!/usr/bin/python

import time
import sys
import json
import roslib
import rospy
from karpal_moveit.karpal_moveit_api import *
from karpal_main.srv import *

def executeMovement(pos):
	moveit = karpal_moveit_api("left_arm")
	moveit.addObject()
	
	moveit.group.set_named_target('left_neutral')
	moveit.group.go()
	
	rospy.sleep(1)
	
	milkPos = pos
	bowlPos = [0.82, 0, -0.07]
	
	picked = moveit.pick(milkPos, preGraspDistance=0.12)
	
	if(picked):
		if (moveit.pour(bowlPos)):
			moveit.place(milkPos)
	
	rospy.sleep(1)
	pose = moveit.moveHelper.get_current_pose()
	pose.position.x -= 0.08
	pose.position.y += 0.08
	moveit.move_to_target_pose(pose)
	
	rospy.sleep(1)
	moveit.group.set_named_target('left_neutral')
	moveit.group.go()


def pickup(item):
	if(item is None):
		return

	print "Getting position for item " + item
	# Block until service is available
	rospy.wait_for_service('get_object_location')
	print "Communicate with the service"
	
	try:
		# Create a handle for the service with the type of service
		getObjPositionService = rospy.ServiceProxy('get_object_location', GetObjectLocation)
		# Get the response by supply the correct parameter
		response = getObjPositionService(item)
		# Get the service output from the response
		objLocation =  response.object_location
		print objLocation
		# Execute movement
		executeMovement(objLocation)

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
