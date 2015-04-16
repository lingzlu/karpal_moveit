#!/usr/bin/env python

import rospy, sys, tf
sys.path.append('/home/course/lu3421/catkin_ws/src/gripping/src')
import moveit_commander
import math
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
import moveit_msgs.msg
from copy import deepcopy
from grasp import Gripper
import baxter_interface


graspOrientations = [
	[0.081,0.065,-0.155,0.705],
	[0.105,0.707,-0.074,0.694],
	[0.106,0.702,-0.102,0.696],
	[0.135,0.683,-0.143,0.702],
	[0.119,0.698,-0.083,0.7],
	[0.045,0.708,-0.080,0.7],
	[0.133,0.705,-0.129,0.68],
]

class karpal_moveit:

	def __init__(self, arm="left_arm"):
		# Initialize the move_group API
		moveit_commander.roscpp_initialize(sys.argv)

		# Initialize the ROS node
		rospy.init_node('karpal_moveit', anonymous=True)

		# Connect to the arm move group
		if arm == "right_arm":
			self.arm = MoveGroupCommander('right_arm')
			self.gripper = Gripper(Gripper.Right)	
		else:
			self.arm = MoveGroupCommander('left_arm')
			self.gripper = Gripper(Gripper.Left)

		rospy.sleep(1)
		self.gripper.release()

		# Allow replanning to increase the odds of a solution
		self.arm.allow_replanning(True)

		# Allow some leeway in position(meters) and orientation (radians)
		self.arm.set_goal_position_tolerance(0.001)
		self.arm.set_goal_orientation_tolerance(0.01)

		# We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
        	#display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

        	#print "Waiting for RVIZ..."
        	rospy.sleep(1)


	def execute_cartesian_path(self, waypoints):

		fraction = 0.0
		maxAttempts = 50
		attempts = 0

		# Set the internal state to the current state
		self.arm.set_start_state_to_current_state()

		# Plan the Cartesian path connecting the waypoints
		while fraction < 1.0 and attempts < maxAttempts:
			(plan, fraction) = self.arm.compute_cartesian_path (
								 waypoints,   # waypoint poses
								 0.01,        # eef_step
								 0.0,         # jump_threshold
								 False)       # avoid_collisions
			# Increment the number of attempts
			attempts += 1

		# If we have a complete plan, execute the trajectory
		if fraction == 1.0:
			rospy.loginfo("Path computed successfully. Moving the arm.")
			self.arm.execute(plan)
			rospy.loginfo("Path execution complete.")
			return True
		else:
			rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxAttempts) + " attempts.")
			return False
	
	def get_current_pose(self):
		end_effector_link = self.arm.get_end_effector_link()
		tfl = tf.TransformListener()
		tfl.waitForTransform("base", end_effector_link, rospy.Time(0), rospy.Duration(4.0))
		(position, orientation) = tfl.lookupTransform("base", end_effector_link, rospy.Time(0))
		current_pose = Pose()
		current_pose.position.x = position[0]
		current_pose.position.y = position[1]
		current_pose.position.z = position[2]
		current_pose.orientation.x = orientation[0]
		current_pose.orientation.y = orientation[1]
		current_pose.orientation.z = orientation[2]
		current_pose.orientation.w = orientation[3]
		return current_pose

	def moveToTargetPose(self, targetPose):
		# move to the defined pose
		self.arm.set_start_state_to_current_state()
		self.arm.set_pose_target(targetPose)
		plan = self.arm.plan()
		return self.arm.execute(plan)

	
	def setOrientation(self, orientation):
		ori = Pose().orientation
		ori.x = orientation[0]
		ori.y = orientation[1]
		ori.z = orientation[2]
		ori.w = orientation[3]
		return ori

	def pickFromSide(self, targetPosition):
		moved = False
		targetPose=Pose()

		targetPose.position.x = targetPosition[0] - 0.1
		targetPose.position.y = targetPosition[1]
		targetPose.position.z = targetPosition[2]
		
		for orientation in graspOrientations:
			targetPose.orientation = self.setOrientation(orientation)
			if(self.moveToTargetPose(targetPose)): 
				moved = True
				break
		if not moved:
			return False

		maxMove = 10
		moveCount = 0
		while self.gripper.range.state() > 70 and moveCount < maxMove:
			targetPose.position.x += 0.02
			self.moveToTargetPose(targetPose)
			moveCount += 1
			rospy.sleep(0.5)

		self.gripper.grasp()
		rospy.sleep(1)
		return True
	
	def place(self, targetPosition):
		currentPose = self.get_current_pose()
		for orientation in graspOrientations:
			waypoints = []
			waypoints.append(currentPose)
	
			currentPose.orientation = self.setOrientation(orientation)

			wPose = deepcopy(currentPose)
			wPose.position.z += 0.10
			waypoints.append(wPose)
			
			wPose = deepcopy(wPose)
			wPose.position.y = targetPosition[1]
			waypoints.append(wPose)
			
			wPose = deepcopy(wPose)
			wPose.position.x = targetPosition[0]
			waypoints.append(wPose)
	
			wPose = deepcopy(wPose)
			wPose.position.z = targetPosition[2]
			waypoints.append(wPose)
	
			if self.execute_cartesian_path(waypoints):
				break
		
		self.gripper.release()
		rospy.sleep(1)
	
	def circular_path(self, cycles=2):
		start_pose = self.get_current_pose()

		waypoints = []

		# Append the pose to the waypoints list
		waypoints.append(start_pose)

		radius = 0.05
		stepSize = math.pi/4

		for i in range(cycles):
			step = 0
			while step*stepSize < 2*math.pi:
				tempPose = deepcopy(start_pose)
				tempX = radius*math.cos(step*stepSize)
				tempY = radius*math.sin(step*stepSize)
				tempPose.position.x += tempX
				tempPose.position.y += tempY
				waypoints.append(tempPose)
				step +=1
		return waypoints

	def testPose(self):
		testRot = [0.135,0.683,-0.143,0.702]
		start_pose = self.get_current_pose()
		start_pose.position.x = 0.8
		start_pose.position.y = 0.4
		start_pose.position.z = -0.05
		start_pose.orientation.x = testRot[0]
		start_pose.orientation.y = testRot[1]
		start_pose.orientation.z = testRot[2]
		start_pose.orientation.w = testRot[3]
		self.moveToTargetPose(start_pose)

if __name__ == "__main__":
	
	# demo: do circular motion on left arm
	moveit = karpal_moveit("left_arm")

	# Move back to the 'neutral' position (defined in SRDF file)
	moveit.arm.set_named_target('left_neutral')
	moveit.arm.go()
	rospy.sleep(1)

	#moveit.testPose()
	
	pick = moveit.pickFromSide([0.78, 0.161, -0.06]) #left
	if(pick):
		moveit.place([0.85, 0.3, -0.06])

	# Shut down MoveIt cleanly
	moveit_commander.roscpp_shutdown()

	# Exit MoveIt
	moveit_commander.os._exit(0)
