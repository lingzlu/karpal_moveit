#!/usr/bin/env python

import rospy, sys
import moveit_commander
import math
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
import moveit_msgs.msg
from copy import deepcopy

class Path_Planning:
	def __init__(self, arm="left_arm"):
		# Initialize the move_group API
		moveit_commander.roscpp_initialize(sys.argv)

		# Initialize the ROS node
		rospy.init_node('moveit_circular_motion', anonymous=True)

		# Connect to the arm move group
		if self.arm == "right_arm":
			self.arm = MoveGroupCommander('right_arm')
		else:
			self.arm = MoveGroupCommander('left_arm')

		# Allow replanning to increase the odds of a solution
		arm.allow_replanning(True)

		# Allow some leeway in position(meters) and orientation (radians)
		arm.set_goal_position_tolerance(0.001)
		arm.set_goal_orientation_tolerance(0.01)

	def compute_cartesian_path(self, waypoints):
		# Get the name of the end-effector link
		end_effector_link = arm.get_end_effector_link()
		# Plan and execute a trajectory to the goal configuration
		self.arm.go()
		rospy.sleep(1)
		# Get the current pose so we can add it as a waypoint
		start_pose = self.arm.get_current_pose(end_effector_link).pose

		fraction = 0.0
		maxtries = 100
		attempts = 0

		# Set the internal state to the current state
		self.arm.set_start_state_to_current_state()

		# Plan the Cartesian path connecting the waypoints
		while fraction < 1.0 and attempts < maxtries:
			(plan, fraction) = arm.compute_cartesian_path (
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
			arm.execute(plan)
			rospy.loginfo("Path execution complete.")
		else:
			rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")

	def circular_path(self):
		waypoints = []
		#wpose = deepcopy(current_pose)
		wpose=Pose()

		wpose.position.x = 0.619027463078
		wpose.position.y = 0.456932535372
		wpose.position.z = 0.35963384769

		wpose.orientation.x = 0.89646727127
		wpose.orientation.y = -0.441927661622
		wpose.orientation.z = -0.0323472356087
		wpose.orientation.w = 0.000172577279596

		# Append the pose to the waypoints list
		waypoints.append(deepcopy(wpose))
		radius = 0.05
		for a in range(0, 28, 1):
			tempX = radius*math.cos(a*math.pi/4)
			tempY = radius*math.sin(a*math.pi/4)
			wpose.position.x += tempX
			wpose.position.y += tempY
			waypoints.append(deepcopy(wpose))

		return waypoints

if __name__ == "__main__":
	cartesianPath = Path_Planning("left_arm")
	wayspoints = cartesianPath.circular_path();
	cartesianPath.compute_cartesian_path(wayspoints)

	# Move normally back to the 'neutral' position
	arm.set_named_target('left_neutral')
	arm.go()
	rospy.sleep(1)

	# Shut down MoveIt cleanly
	moveit_commander.roscpp_shutdown()

	# Exit MoveIt
	moveit_commander.os._exit(0)
