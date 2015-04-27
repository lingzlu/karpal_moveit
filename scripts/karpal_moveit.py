#!/usr/bin/env python

import rospy, sys, tf
sys.path.append('/home/course/lu3421/catkin_ws/src/gripping/src')
import moveit_commander
import math
from moveit_commander import MoveGroupCommander, RobotCommander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point

from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from copy import deepcopy
from grasp import Gripper
import baxter_interface
from move_helper import MoveHelper
import numpy as np
import random

graspOrientations = [
	[0.347,0.651,-0.316,0.596],
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
		rospy.init_node('karpal_moveit_test', anonymous=True)
		
		self.scene = PlanningSceneInterface()
		self.robot = RobotCommander()	
		self.tfl = tf.TransformListener()

		# Connect to the arm move group
		if arm == "right_arm":
			#self.group = MoveGroupCommander('right_arm')
			#self.gripper = Gripper(Gripper.Right)
			#self.gripper = baxter_interface.Gripper('right')
			#moveHelper = MoveHelper('right')
			self.limb = "right"
		else:
			#self.group = MoveGroupCommander('left_arm')
			#self.gripper = baxter_interface.Gripper('left')
			self.limb = "left"	
		
		self.group = MoveGroupCommander(self.limb + '_arm')
		self.gripper = baxter_interface.Gripper(self.limb)
		self.gripperSensor = baxter_interface.AnalogIO(self.limb + '_hand_range')
		self.moveHelper = MoveHelper(self.limb)

		self.group.set_start_state_to_current_state()
		#self.gripper.calibrate()

		#rospy.sleep(1)
		self.gripper.open()

		# Allow replanning to increase the odds of a solution
		self.group.allow_replanning(True)

		# Allow some leeway in position(meters) and orientation (radians)
		self.group.set_goal_position_tolerance(0.005)
		self.group.set_goal_orientation_tolerance(0.05)

		# We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
        	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=10)

        	#print "Waiting for RVIZ..."
        	rospy.sleep(1)

	def execute_cartesian_path(self, waypoints):

		fraction = 0.0
		maxAttempts = 50
		attempts = 0

		# Set the internal state to the current state
		self.group.set_start_state_to_current_state()

		# Plan the Cartesian path connecting the waypoints
		while fraction < 1.0 and attempts < maxAttempts:
			(plan, fraction) = self.group.compute_cartesian_path (
								 waypoints,   # waypoint poses
								 0.01,        # eef_step
								 0.0,         # jump_threshold
								 False)       # avoid_collisions
			# Increment the number of attempts
			attempts += 1

		# If we have a complete plan, execute the trajectory
		if fraction == 1.0:
			rospy.loginfo("Path computed successfully. Moving the arm.")
			self.group.execute(plan)
			rospy.loginfo("Path execution complete.")
			return True
		else:
			rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxAttempts) + " attempts.")
			return False
	
	def moveToTargetPose(self, targetPose):
		# move to the defined pose
		self.group.set_start_state_to_current_state()
		self.group.set_pose_target(targetPose)
		plan = self.group.plan()
		return self.group.execute(plan)



	def stir(self, cycles = 3):
		start_pose = self.movehelper.get_current_pose()
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

		self.execute_cartesian_path(waypoints)


	def pour(self):
		""" Method to rotate Baxter's wrist, which will pour out
			contents from a cup or similar object it is gripping.
			@param self The object pointer
		"""

		# remember starting position
		currentPose = self.moveHelper.get_current_pose()
		targetPose = deepcopy(currentPose)

		# remember starting angles
		startingAngles = self.moveHelper.get_current_joint_values()

		# decide whether to pour clockwise or counterclockwise
		targetAngles = deepcopy(startingAngles)

		gripperAngle = startingAngles[self.limb + "_w2"]
		if gripperAngle < 0.05:
			targetPose.position.y += 0.05
			targetAngles[self.limb + "_w2"] += 2.5
		else:
			targetPose.position.y -= 0.05
			targetAngles[self.limb + "_w2"] -= 2.5

		# move to optimal pouring position
		self.moveToTargetPose(targetPose)
		rospy.sleep(1)
		
		self.group.set_joint_value_target(targetAngles)
		self.group.go()
		rospy.sleep(3)

		self.group.set_joint_value_target(startingAngles)
		self.group.go()
		rospy.sleep(1)
	
	def pickFromSide(self, targetPosition):
		moved = False
		targetPose=Pose()

		targetPose.position.x = targetPosition[0] - 0.1
		targetPose.position.y = targetPosition[1]
		targetPose.position.z = targetPosition[2]
		
		for orientation in graspOrientations:
			targetPose.orientation = Quaternion(*orientation)
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
		self.moveAboveObject(targetPosition, 0)
		rospy.sleep(1)
		currentPose = self.moveHelper.get_current_pose()
		targetPose = deepcopy(currentPose)
		targetPose.position.z -= 0.2
		self.moveToTargetPose(targetPose)
		rospy.sleep(1)

		self.gripper.open()
		rospy.sleep(1)
	
	def moveAboveObject(self, targetPosition, height=0.20):
		""" Moves the gripper above the position of a object.
			The orientation is kept so that anything held will stay upright.
			@param targetPosition The position of target object in the form of array [x y z]
		"""
		currentPose = self.moveHelper.get_current_pose()
		isMoved = False
		
		for orientation in graspOrientations:
			waypoints = []
			waypoints.append(currentPose)

			wPose = deepcopy(currentPose)
			wPose.position.z += height/2
			waypoints.append(wPose)
	
			wPose = deepcopy(wPose)
			wPose.orientation = Quaternion(*orientation)
			wPose.position.z += height/2
			waypoints.append(wPose)

			wPose = deepcopy(wPose)
			wPose.position.x = targetPosition[0]
			waypoints.append(wPose)

			wPose = deepcopy(wPose)
			wPose.position.y = targetPosition[1]
			waypoints.append(wPose)
			
			if self.execute_cartesian_path(waypoints):
				break

		rospy.sleep(1)
		"""
		firstAttempt = True
		while not isMoved:
			waypoints = []
			waypoints.append(currentPose)
			
			if not firstAttempt:
				orien = self.moveHelper.get_current_orientation()
				newOrien = self.generateOrientation(orien)
				currentPose.orientation = Quaternion(*newOrien)

			# 20 cm above the target object
			wPose = deepcopy(currentPose)
			wPose.position.z += height
			waypoints.append(wPose)
			
			wPose = deepcopy(wPose)
			wPose.position.x = targetPosition[0]
			waypoints.append(wPose)

			wPose = deepcopy(wPose)
			wPose.position.y = targetPosition[1]
			waypoints.append(wPose)
			
			if self.execute_cartesian_path(waypoints):
				isMoved = True
				break
			firstAtempt = False
			rospy.sleep(1)
		"""
		
	def addObject(self):
		self.scene.remove_world_object("table")
		self.scene.remove_world_object("part")

		# publish a demo scene
    		p = PoseStamped()
    		p.header.frame_id = self.robot.get_planning_frame()

		# add a table
	        p.pose.position.x = 0.85
		p.pose.position.y = -0.1
		p.pose.position.z = -0.26
    		self.scene.add_box("table", p, (1.1, 1.1, 0.2))
		"""
		# add an object to be grasped
	    	p.pose.position.x = 0.9
		p.pose.position.y = 0.2
		p.pose.position.z = 0.03
		p.pose.orientation.w = 0.5
		self.scene.add_box("box", p, (0.05, 0.05, 0.15))
		"""

	def generateOrientation(self, orientation):	
		deg = random.uniform(-15,15)
		print deg
		rot = tf.transformations.quaternion_from_euler(0, 0, deg*math.pi/180)
		
		newOrien = tf.transformations.quaternion_multiply(rot, orientation)
		return newOrien

	def generateGrasp(self, sample_pose):
	
		orien = []

		orien.append(sample_pose.orientation.x)
		orien.append(sample_pose.orientation.y)
		orien.append(sample_pose.orientation.z)
		orien.append(sample_pose.orientation.w)
		
		newOrien = self.generateOrientation(orien)

		targetPose = deepcopy(sample_pose)
		
		targetPose.position.x -= 0.08
		targetPose.position.y += 0.08
	
		targetPose.orientation = Quaternion(*newOrien)

		print targetPose
		return targetPose

	def pick(self, targetPosition):
		samplePose=Pose()
		samplePose.position.x = targetPosition[0]
		samplePose.position.y = targetPosition[1]
		samplePose.position.z = targetPosition[2]
		samplePose.orientation.x = 0.278
		samplePose.orientation.y = 0.678
		samplePose.orientation.z = -0.19
		samplePose.orientation.w = 0.653
		
		count = 0
		isMoved = False
		while not isMoved and count < 5:
			graspPose = self.generateGrasp(samplePose)
			isMoved = self.moveToTargetPose(graspPose)
			count += 1

		if not isMoved:
			return False

		rospy.sleep(1)
		
		
		preGraspPoint = np.array([graspPose.position.x, graspPose.position.y])
		targetGraspPoint = np.array(targetPosition[0:2])
		vector = targetGraspPoint - preGraspPoint
		normVector = vector/np.linalg.norm(vector)
		
		maxMove = 12
		moveCount = 0
		while self.gripperSensor.state() > 70 and moveCount < maxMove:
			graspPose.position.x += 0.02*normVector[0]
			graspPose.position.y += 0.02*normVector[1]
			self.moveToTargetPose(graspPose)
			moveCount += 1
			rospy.sleep(0.5)
		
		self.gripper.close()
		rospy.sleep(1)
		return True

	def test(self):
		currentPose = self.moveHelper.get_current_pose()

		targetPose = deepcopy(currentPose)
		orien = []
		orien.append(currentPose.orientation.x)
		orien.append(currentPose.orientation.y)
		orien.append(currentPose.orientation.z)
		orien.append(currentPose.orientation.w)
		
		orien_euler1 = tf.transformations.euler_from_quaternion(orien)
		print orien_euler1

		rot = tf.transformations.quaternion_from_euler(0, 0, -10*math.pi/180)
		newOrien = tf.transformations.quaternion_multiply(rot, orien)
		
		targetPose.orientation = Quaternion(*newOrien)
		orien_euler = tf.transformations.euler_from_quaternion(newOrien)
		print orien_euler
		self.moveToTargetPose(targetPose)
		
if __name__ == "__main__":

	# demo: do circular motion on left armup.set_named_target(limb + "_neutral")
	moveit = karpal_moveit("left_arm")
	# Move back to the 'neutral' position (defined in SRDF file)
	moveit.group.set_named_target('left_neutral')
	moveit.group.go()
	
	rospy.sleep(2)

	moveit.addObject()
	#moveit.test()
	picked = moveit.pick([0.63, 0.265, -0.055])
	if(picked):
		moveit.moveAboveObject([0.68, -0.12, -0.06],0.25)
		moveit.pour()
		moveit.place([0.79, 0.265, -0.055])



	# Shut down MoveIt cleanly
	#moveit_commander.roscpp_shutdown()

	# Exit MoveIt
	#moveit_commander.os._exit(0)
