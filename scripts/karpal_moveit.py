#!/usr/bin/env python

import rospy, sys, tf
import moveit_commander
import math
from moveit_commander import MoveGroupCommander, RobotCommander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point

from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from copy import deepcopy
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

		if not self.gripper.calibrated():	
			self.gripper.calibrate()

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
			rospy.sleep(1)
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
		start_pose = self.moveHelper.get_current_pose()
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
		rospy.sleep(2)
	
	def change_waypoints_orientation(self, waypoints):
		currOrien = self.moveHelper.get_current_orientation()
		if self.limb == 'right':
			rot = tf.transformations.quaternion_from_euler(0, 0, -15*math.pi/180)
		else:
			rot = tf.transformations.quaternion_from_euler(0, 0, 15*math.pi/180)
		
		pose = waypoints[-1]
		lastTriedOrien = []
		lastTriedOrien.append(pose.orientation.x)
		lastTriedOrien.append(pose.orientation.y)
		lastTriedOrien.append(pose.orientation.z)
		lastTriedOrien.append(pose.orientation.w)
		newOrien = tf.transformations.quaternion_multiply(rot, lastTriedOrien)
		newOrien = Quaternion(*newOrien)

		for pose in waypoints[1:]:
			pose.orientation = newOrien
		return waypoints
		
	def pour(self, targetPosition):
		""" Method to rotate Baxter's wrist, which will pour out
			contents from a cup or similar object it is gripping.
			@param self The object pointer
		"""

		# remember starting position
		#currentPose = self.moveHelper.get_current_pose()
		#targetPose = deepcopy(currentPose)

		# remember starting angles
		currentJoints = self.moveHelper.get_current_joint_values()

		gripperAngle = currentJoints[self.limb + "_w2"]
		if gripperAngle < 0.05:
			moveToPour = 0.05
			gripperRotation = 2.5

		else:
			moveToPour = -0.05
			gripperRotation = -2.5

		
		waypoints = self.get_waypoints_above_object(targetPosition, 0.2)
		lastPose = deepcopy(waypoints[-1])
		
		lastPose.position.y += moveToPour
		waypoints.append(lastPose)
		movedAboveObject = self.execute_cartesian_path(waypoints)

		count = 0
		while not movedAboveObject and count < 5:
			waypoints = self.change_waypoints_orientation(waypoints)
			print waypoints
			movedAboveObject = self.execute_cartesian_path(waypoints)
			count += 1
		

		print waypoints
		if not movedAboveObject:
			return False

		# move to optimal pouring position
		#self.moveToTargetPose(targetPose)
	
		currentJoints = self.moveHelper.get_current_joint_values()
		targetJoints = deepcopy(currentJoints)
		targetJoints[self.limb + "_w2"] += gripperRotation

		rospy.sleep(2)
		self.group.set_joint_value_target(targetJoints)
		self.group.go()
		rospy.sleep(4)

		self.group.set_joint_value_target(currentJoints)
		self.group.go()
		rospy.sleep(1)
		return True
	
	def place(self, targetPosition):
		waypoints = self.get_waypoints_above_object(targetPosition, 0)
		lastPose = deepcopy(waypoints[-1])
		
		lastPose.position.z = targetPosition[2] + 0.02
		waypoints.append(lastPose)
		moved = self.execute_cartesian_path(waypoints)

		self.gripper.open()
		rospy.sleep(1)
		return moved
	
	def get_waypoints_above_object(self, targetPosition, height=0.15):
		currentPose = self.moveHelper.get_current_pose()
		waypoints = []
		waypoints.append(currentPose)
		
		wPose = deepcopy(currentPose)
		wPose.position.z += height/2
		waypoints.append(wPose)
		
		wPose = deepcopy(wPose)
		"""
		if self.limb == 'right':
			deg = -10 * count
		else:
			deg = 10 * count

		rot = tf.transformations.quaternion_from_euler(0, 0, deg*math.pi/180)
		orrosrun baxter_tools enable_robot.py -eientation = self.moveHelper.get_current_orientation()
		newOrien = tf.transformations.quaternion_multiply(rot, orientation)
		wPose.orientation = Quaternion(*newOrien)
		"""
		wPose.position.z += height/2
		waypoints.append(wPose)
		
		wPose = deepcopy(wPose)
		wPose.position.x = targetPosition[0]
		waypoints.append(wPose)

		wPose = deepcopy(wPose)
		wPose.position.y = targetPosition[1]
		waypoints.append(wPose)

		return waypoints

	def move_above_object(self, targetPosition, height=0.15):
		""" Moves the gripper above the position of a object.
			The orientation is kept so that anything held will stay upright.
			@param targetPosition The position of target object in the form of array [x y z]
		"""
		currentPose = self.moveHelper.get_current_pose()
		isMoved = False
		
		"""
		for orientation in graspOrientations:
			waypoints = []
			waypoints.append(currentPose)

			wPose = deepcopy(currentPose)
			wPose.position.z += height/2
			waypoints.append(wPose)
	
			wPose = deepcopy(wPose)
			wPose.orientation = Quaternion(*orientation)
			wPose.position.z += height/2 + 0.5
			waypoints.append(wPose)

			wPose = deepcopy(wPose)
			wPose.position.x = targetPosition[0]
			waypoints.append(wPose)

			wPose = deepcopy(wPose)
			wPose.position.y = targetPosition[1]
			waypoints.append(wPose)
			
			wPose = deepcopy(wPose)
			wPose.position.z -= 0.5
			waypoints.append(wPose)

			if self.execute_cartesian_path(waypoints):
				break

		rospy.sleep(1)
		"""

		count = 0
		while not isMoved and count < 5:
			
			wayspoints = self.get_waypoints_above_object(targetPosition, height)
			if self.execute_cartesian_path(waypoints):
				isMoved = True
				break
			count += 1

		rospy.sleep(2)
		return isMoved
		
	def addObject(self):
		self.scene.remove_world_object("table")
		self.scene.remove_world_object("wall")
		self.scene.remove_world_object("camera")

		# publish a demo scene
    		p = PoseStamped()
    		p.header.frame_id = self.robot.get_planning_frame()

		# add a table
	        p.pose.position.x = 0.85
		p.pose.position.y = -0.1
		p.pose.position.z = -0.26
    		self.scene.add_box("table", p, (1.1, 1.1, 0.2))

		# add camera
	        p.pose.position.x = 0.16
		p.pose.position.y = 0
		p.pose.position.z = 0.54
    		self.scene.add_box("camera", p, (0.04, 0.2, 0.04))
		
	    	p.pose.position.x = 0.35
		p.pose.position.y = -1.15
		p.pose.position.z = -0.15
		self.scene.add_box("wall", p, (0.1, 0.6, 0.4))

		
	def generateOrientation(self, orientation):	
		deg = random.uniform(-20,20)
		print deg
		rot = tf.transformations.quaternion_from_euler(0, 0, deg*math.pi/180)
		
		newOrien = tf.transformations.quaternion_multiply(rot, orientation)
		return newOrien

	def generate_pregrasp_pose(self, pickPose, preGraspDistance = 0.08, fixedOrientation = False):
	
		preGraspPose = deepcopy(pickPose)
		if not fixedOrientation:
			orien = []
			orien.append(pickPose.orientation.x)
			orien.append(pickPose.orientation.y)
			orien.append(pickPose.orientation.z)
			orien.append(pickPose.orientation.w)
			newOrien = self.generateOrientation(orien)
			preGraspPose.orientation = Quaternion(*newOrien)

		if self.limb == 'left':
			preGraspPose.position.x -= preGraspDistance
			preGraspPose.position.y += preGraspDistance
		else:
			preGraspPose.position.x -= preGraspDistance
			preGraspPose.position.y -= preGraspDistance
		print preGraspPose
		return preGraspPose

	def pick(self, position, orientation = None, preGraspDistance = 0.08, fixedOrientation = False):

		if orientation == None:
			orientation = [0.278, 0.678, -0.19, 0.653]
			if self.limb == 'right':
				rot = tf.transformations.quaternion_from_euler(0, 0, 90*math.pi/180)
				orientation = tf.transformations.quaternion_multiply(rot, orientation)
		#print "orientation is: " , orientation
		pickPose = Pose()
		pickPose.position = Point(*position)
		pickPose.orientation = Quaternion(*orientation)

		isMoved = False
		
		if fixedOrientation:
			preGraspPose = self.generate_pregrasp_pose(pickPose, preGraspDistance, fixedOrientation)
			isMoved = self.moveToTargetPose(preGraspPose)
		else:
			count = 0
			while not isMoved and count < 5:
				preGraspPose = self.generate_pregrasp_pose(pickPose, preGraspDistance, fixedOrientation)
				isMoved = self.moveToTargetPose(preGraspPose)
			count += 1

		if not isMoved:
			return False

		rospy.sleep(1)
		
		
		preGraspPoint = np.array([preGraspPose.position.x, preGraspPose.position.y])
		targetGraspPoint = np.array(position[0:2])
		vector = targetGraspPoint - preGraspPoint
		
		normVector = vector/np.linalg.norm(vector)
		print targetGraspPoint, preGraspPoint
		print normVector

		maxMove = 12
		moveCount = 0
		while self.gripperSensor.state() > 75 and moveCount < maxMove:

			if self.limb == 'left':
				#preGraspPose.position.x += 0.02
				#preGraspPose.position.y -= 0.02
				preGraspPose.position.x += 0.02*normVector[0]
				preGraspPose.position.y += 0.02*normVector[1]
			else:
				#preGraspPose.position.x += 0.02
				#preGraspPose.position.y += 0.02
				preGraspPose.position.x += 0.02*normVector[0]
				preGraspPose.position.y += 0.02*normVector[1]
			
			self.moveToTargetPose(preGraspPose)
			moveCount += 1
			rospy.sleep(2)
		
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
		print orien

		rot = tf.transformations.quaternion_from_euler(0, 0, -10*math.pi/180)
		newOrien = tf.transformations.quaternion_multiply(rot, orien)
		
		targetPose.orientation = Quaternion(*newOrien)
		orien_euler = tf.transformations.euler_from_quaternion(newOrien)
		print newOrien
		self.moveToTargetPose(targetPose)
	
def testLeftArm():
	moveit = karpal_moveit("left_arm")
	#moveit.test()

	moveit.addObject()

	moveit.group.set_named_target('left_neutral')
	moveit.group.go()
	
	rospy.sleep(2)
	
	objectPosition = [0.79, 0.265, -0.055]
	pourPosition = [0.68, -0.12, -0.06]
	pickOrientation = [0.278, 0.678, -0.19, 0.653]

	picked = moveit.pick(objectPosition, pickOrientation)

	if(picked):
		if (moveit.pour(pourPosition)):
			moveit.place(objectPosition)

def testRightArm():
	moveit = karpal_moveit("right_arm")
	# Move back to the 'neutral' position (defined in SRDF file)
	moveit.addObject()

	moveit.group.set_named_target('right_neutral')
	moveit.group.go()
	
	rospy.sleep(2)
	
	objectPosition = [0.732, -0.361, 0.0681]
	pickOrientation = [-0.141, 0.684, 0.075, 0.711]
	#moveit.test()
	
	picked = moveit.pick(objectPosition, preGraspDistance = 0.05, fixedOrientation = True)
	if(picked):
		if moveit.move_above_object([0.68, -0.12, -0.06], 0.20):
			moveit.stir(5)
			#moveit.pour()
			#moveit.place(objectPosition)
	
if __name__ == "__main__":
	testLeftArm()
	#testRightArm()


	# Shut down MoveIt cleanly
	#moveit_commander.roscpp_shutdown()

	# Exit MoveIt
	#moveit_commander.os._exit(0)
