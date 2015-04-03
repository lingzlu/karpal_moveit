#!/usr/bin/env python

import rospy, sys
import moveit_commander
import math
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
import moveit_msgs.msg
from copy import deepcopy

class Path_Planning:

	def __init__(self, planArm="left_arm"):
		# Initialize the move_group API
		moveit_commander.roscpp_initialize(sys.argv)

		# Initialize the ROS node
		rospy.init_node('moveit_circular_motion', anonymous=True)

		# Connect to the arm move group
		if planArm == "right_arm":
			self.arm = MoveGroupCommander('right_arm')
		else:
			self.arm = MoveGroupCommander('left_arm')

		# Allow replanning to increase the odds of a solution
		self.arm.allow_replanning(True)

		# Allow some leeway in position(meters) and orientation (radians)
		self.arm.set_goal_position_tolerance(0.001)
		self.arm.set_goal_orientation_tolerance(0.01)

	def compute_cartesian_path(self, waypoints):

		fraction = 0.0
		maxtries = 100
		attempts = 0

		# Set the internal state to the current state
		self.arm.set_start_state_to_current_state()

		# Plan the Cartesian path connecting the waypoints
		while fraction < 1.0 and attempts < maxtries:
			(plan, fraction) = self.arm.compute_cartesian_path (
								 waypoints,   # waypoint poses
								 0.01,        # eef_step
								 0.0,         # jump_threshold
								 False)       # avoid_collisions
			# Increment the number of attempts
			attempts += 1

			# Print out a progress message
			if attempts % 10 == 0:
				rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

		# If we have a complete plan, execute the trajectory
		if fraction == 1.0:
			rospy.loginfo("Path computed successfully. Moving the arm.")
			self.arm.execute(plan)
			rospy.loginfo("Path execution complete.")
		else:
			rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")

	def circular_path(self, cycles=2):
		# Get the name of the end-effector link
		end_effector_link = self.arm.get_end_effector_link()

		# Get the current pose so we can add it as a waypoint
		start_pose = self.arm.get_current_pose(end_effector_link).pose
		print "start pose\n", start_pose
		waypoints = []
		
		wpose=Pose()
		wpose.position.x = 0.741
		wpose.position.y = 0.478
		wpose.position.z = 0.153
	
		wpose.orientation.x = -0.4644
		wpose.orientation.y = 0.8838
		wpose.orientation.z = 0.0157
		wpose.orientation.w = 0.0534

		# Append the pose to the waypoints list
		waypoints.append(deepcopy(wpose))

		radius = 0.05
		stepSize = math.pi/4

		for i in range(cycles):
			step = 0
			while step*stepSize < 2*math.pi:
				tempPose = deepcopy(wpose)
				tempX = radius*math.cos(step*stepSize)
				tempY = radius*math.sin(step*stepSize)
				tempPose.position.x += tempX
				tempPose.position.y += tempY
				waypoints.append(tempPose)
				step +=1
		return waypoints

if __name__ == "__main__":
	
	# demo: do circular motion on left arm

	cartesianPath = Path_Planning("left_arm")
	
	# define a starting pose
	wpose=Pose()
	wpose.position.x = 0.741
	wpose.position.y = 0.478
	wpose.position.z = 0.153
	wpose.orientation.x = -0.4644
	wpose.orientation.y = 0.8838
	wpose.orientation.z = 0.0157
	wpose.orientation.w = 0.0534

	# move to the defined pose
	cartesianPath.arm.set_pose_target(wpose)
	cartesianPath.arm.go()
	rospy.sleep(1)
	
	# get the circular path waypoints
	wayspoints = cartesianPath.circular_path();

	# move the arm following the wayspoints
	cartesianPath.compute_cartesian_path(wayspoints)
	rospy.sleep(1)

	# Move back to the 'neutral' position (defined in SRDF file)
	cartesianPath.arm.set_named_target('left_neutral')
	cartesianPath.arm.go()
	rospy.sleep(1)

	# Shut down MoveIt cleanly
	moveit_commander.roscpp_shutdown()

	# Exit MoveIt
	moveit_commander.os._exit(0)
