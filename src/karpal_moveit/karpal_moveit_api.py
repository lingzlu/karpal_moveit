#!/usr/bin/env python

import rospy, sys, tf
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point

from moveit_msgs.msg import Grasp, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import baxter_interface

from copy import deepcopy
import numpy, random, math
from move_helper import MoveHelper

class KarpalMoveit:

	def __init__(self, arm="left_arm"):
		# Initialize the move_group API
		moveit_commander.roscpp_initialize(sys.argv)

		# Initialize the ROS node
		rospy.init_node('KarpalMoveit_test', anonymous=True)

		self.scene = moveit_commander.PlanningSceneInterface()
		self.robot = moveit_commander.RobotCommander()
		self.tfl = tf.TransformListener()

		# Connect to the arm move group
		if arm == "right_arm":
			self.limb = "right"
		else:
			self.limb = "left"

		# create a moveit group arm object
		self.group = moveit_commander.MoveGroupCommander(self.limb + '_arm')

		# using Baxter Interafce API for interacting with the gripper
		self.gripper = baxter_interface.Gripper(self.limb)

		# instantiated range sensor on the hand
		self.handSensor = baxter_interface.AnalogIO(self.limb + '_hand_range')

		# set arm for the move helper class
		MoveHelper.set_arm(self.limb)

		# calibrate gripper
		if not self.gripper.calibrated():
			self.gripper.calibrate()

		# open gripper initially
		rospy.sleep(1)
		self.gripper.open()

		# Allow replanning to increase the odds of a solution
		self.group.allow_replanning(True)

		# Allow some leeway in position(meters) and orientation (radians)
		self.group.set_goal_position_tolerance(0.005)
		self.group.set_goal_orientation_tolerance(0.05)

		# We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
        	#display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=10)

        	#print "Waiting for RVIZ..."
       		#rospy.sleep(1)

	def execute_cartesian_path(self, waypoints):
		"""
		Execute a sequence of waypoints that make end-effector move in straight line segments that follow
		the poses specified as waypoints. The waypoints are modified when path planning failed, the
		end-effector orientation is rotated about z-axis at some degree in hope of finding a computation

		@param waypoints A list of geometry_msgs.msg.Pose
		@return Returns true if path computed successfully
		"""

		maxAttempts = 50
		attempts = 0

		# Set the internal state to the current state
		self.group.set_start_state_to_current_state()

		pathComputed = False

		while not pathComputed and attempts < maxAttempts:
			(plan, fraction) = self.group.compute_cartesian_path (
					 waypoints,   # waypoint poses
					 0.01,        # eef_step
					 0.0,         # jump_threshold
					 False)       # avoid_collisions

			if fraction == 1.0:
				rospy.loginfo("Path computed successfully. Moving the arm.")
				self.group.execute(plan)
				rospy.sleep(1)
				return True
			else:
				# modify waypoints orientation
				waypoints = self.change_waypoints_orientation(waypoints)

			attempts += 1

		rospy.loginfo("Path planning failed with " + str(fraction) + " success after " + str(maxAttempts) + " attempts.")
		return False

	def pick(self, position, orientation = None, preGraspDistance = 0.08, fixedOrientation = False):
		"""
		Pick up a object using range sensor on the hand
		@param position The position of object to be grasp in the form of [x y z]. Required parameter
		@param orientation The target pose for the end-effector
		@param preGraspDistance The target pose for the end-effector
		@param fixedOrientation The target pose for the end-effector
		"""

		if orientation == None:
			#orientation = [0.278, 0.678, -0.19, 0.653]
			orientation = [0.225, 0.676, -0.217, 0.666]
			if self.limb == 'right':
				rot = tf.transformations.quaternion_from_euler(0, 0, 90*math.pi/180)
				orientation = tf.transformations.quaternion_multiply(rot, orientation)
		#print "orientation is: " , orientation1
		pickPose = Pose()
		pickPose.position = Point(*position)
		pickPose.orientation = Quaternion(*orientation)

		isMoved = False

		if fixedOrientation:
			preGraspPose = self.generate_pregrasp_pose(pickPose, preGraspDistance, fixedOrientation)
			isMoved = MoveHelper.move_to_target_pose(preGraspPose)
		else:
			count = 0
			while not isMoved and count < 20:
				preGraspPose = self.generate_pregrasp_pose(pickPose, preGraspDistance, fixedOrientation)
				isMoved = MoveHelper.move_to_target_pose(preGraspPose)
			count += 1

		if not isMoved:
			return False

		rospy.sleep(1)


		preGraspPoint = numpy.array([preGraspPose.position.x, preGraspPose.position.y])
		targetGraspPoint = numpy.array(position[0:2])
		vector = targetGraspPoint - preGraspPoint

		normVector = vector/numpy.linalg.norm(vector)
		print targetGraspPoint, preGraspPoint
		print normVector

		xDiff = position[0] - preGraspPose.position.x
		yDiff = position[1]-preGraspPose.position.y
		dist = math.hypot(xDiff, yDiff)

		maxMove = 10
		moveCount = 0
		while self.handSensor.state() > 70 and moveCount < maxMove:
			print self.handSensor.state()
			preGraspPose.position.x += 0.02/dist*xDiff
			preGraspPose.position.y += 0.02/dist*yDiff

			MoveHelper.move_to_target_pose(preGraspPose)
			moveCount += 1
			rospy.sleep(1)

		self.gripper.close()
		rospy.sleep(1)
		return True

	def place(self, targetPosition, moveUp = 0, releaseHeight = 0.02):
		waypoints = self.get_waypoints_above_object(targetPosition, moveUp, releaseHeight)

		#placePose = deepcopy(waypoints[-1])
		#placePose.position.z = targetPosition[2] + 0.02
		#waypoints.append(placePose)

		placed = self.execute_cartesian_path(waypoints)

		self.gripper.open()
		rospy.sleep(1)
		return placed

	def change_waypoints_orientation(self, waypoints):

		# if self.limb == 'right':
		# 	rot = tf.transformations.quaternion_from_euler(0, 0, -15*math.pi/180)
		# else:
		# 	rot = tf.transformations.quaternion_from_euler(0, 0, 15*math.pi/180)

		pose = waypoints[-1]
		waypointsOrien = []
		waypointsOrien.append(pose.orientation.x)
		waypointsOrien.append(pose.orientation.y)
		waypointsOrien.append(pose.orientation.z)
		waypointsOrien.append(pose.orientation.w)

		newOrien = self.generate_orientation(waypointsOrien, 20)
		newOrien = Quaternion(*newOrien)

		for pose in waypoints[1:]:
			pose.orientation = newOrien
		return waypoints

	def generate_orientation(self, orientation, degreeRange = 35):
		deg = random.uniform(-degreeRange, degreeRange)
		print deg
		rot = tf.transformations.quaternion_from_euler(0, 0, deg*math.pi/180)
		
		newOrien = tf.transformations.quaternion_multiply(rot, orientation)
		return newOrien

	def pour(self, targetPosition):
		""" Method to rotate Baxter's wrist, which will pour out
			contents from a cup or similar object it is gripping.
			@param self The object pointer
		"""

		# remember starting position
		#currentPose = MoveHelper.get_current_pose()
		#targetPose = deepcopy(currentPose)

		# remember starting angles
		currentJoints = MoveHelper.get_current_joint_values()

		gripperAngle = currentJoints[self.limb + "_w2"]
		if gripperAngle < 0.05:
			#targetPosition[0] += 0.1*math.cos(angleAboutZ)
			#targetPosition[1] += 0.1*math.sin(angleAboutZ)
			moveToPour = 0.08
			gripperRotation = 2.5

		else:
			moveToPour = -0.08
			gripperRotation = -2.5


		waypoints = self.get_waypoints_above_object(targetPosition, moveUp=0.18, aboveObject=0.18)
		pourPose = deepcopy(waypoints[-1])
		
		pourPose.position.x += moveToPour
		waypoints.append(pourPose)
		movedAboveObject = self.execute_cartesian_path(waypoints)

		# count = 0
		# while not movedAboveObject and count < 5:
		# 	waypoints = self.change_waypoints_orientation(waypoints)
		# 	print waypoints
		# 	movedAboveObject = self.execute_cartesian_path(waypoints)
		# 	count += 1


		# print waypoints
		if not movedAboveObject:
			return False

		# move to optimal pouring position
		#MoveHelper.move_to_target_pose(targetPose)

		currentJoints = MoveHelper.get_current_joint_values()
		targetJoints = deepcopy(currentJoints)
		targetJoints[self.limb + "_w2"] += gripperRotation

		self.group.set_joint_value_target(targetJoints)
		self.group.go()
		rospy.sleep(3)

		self.group.set_joint_value_target(currentJoints)
		self.group.go()
		rospy.sleep(3)
		return True

	def stir(self, radius = 0.05, cycles = 3):
		"""
		Move end-effector in a circular motion
		@param radius Radius of circular path in meter, default = 5 mm
		@param cycles Number of cycles movement, default = 3 cycles
		"""

		start_pose = MoveHelper.get_current_pose()
		waypoints = []

		# Append the pose to the waypoints list
		waypoints.append(start_pose)

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
		print waypoints
		self.execute_cartesian_path(waypoints)
		rospy.sleep(2)

	def get_waypoints_above_object(self, targetPosition, moveUp = 0.2, aboveObject = 0.15):
		currentPose = MoveHelper.get_current_pose()
		waypoints = []

		wPose = deepcopy(currentPose)
		waypoints.append(wPose)

		wPose = deepcopy(wPose)
		wPose.position.z += moveUp
		waypoints.append(wPose)

		wPose = deepcopy(wPose)
		wPose.position.x = targetPosition[0]
		waypoints.append(wPose)

		wPose = deepcopy(wPose)
		wPose.position.y = targetPosition[1]
		waypoints.append(wPose)

		wPose = deepcopy(wPose)
		wPose.position.z = targetPosition[2] + aboveObject
		waypoints.append(wPose)

		return waypoints

	def move_above_object(self, targetPosition, moveUp = 0.15, aboveObject=0.15):
		""" Moves the gripper above the position of a object.
			The orientation is kept so that anything held will stay upright.
			@param targetPosition The position of target object in the form of array [x y z]
		"""

  		waypoints = self.get_waypoints_above_object(targetPosition, moveUp, aboveObject)
		movedAboveObject = self.execute_cartesian_path(waypoints)

  # 		count = 0
		# while not movedAboveObject and count < 5:
		# 	waypoints = self.change_waypoints_orientation(waypoints)
		# 	print waypoints
		# 	movedAboveObject = self.execute_cartesian_path(waypoints)
		# 	count += 1

		if movedAboveObject:
			rospy.sleep(1)
			return True

		return False

	def generate_pregrasp_pose(self, pickPose, preGraspDistance = 0.08, fixedOrientation = False):
		preGraspPose = deepcopy(pickPose)
		if not fixedOrientation:
			orien = []
			orien.append(pickPose.orientation.x)
			orien.append(pickPose.orientation.y)
			orien.append(pickPose.orientation.z)
			orien.append(pickPose.orientation.w)
			newOrien = self.generate_orientation(orien)
			preGraspPose.orientation = Quaternion(*newOrien)

		if self.limb == 'left':
			preGraspPose.position.x -= preGraspDistance
			preGraspPose.position.y += preGraspDistance
		else:
			preGraspPose.position.x -= preGraspDistance
			preGraspPose.position.y -= preGraspDistance
		print preGraspPose
		return preGraspPose
	
	def quaternion_to_axis_angle(self, orientation):
		qx = orientation[0]
		qy = orientation[1]
		qz = orientation[2]
		qw = orientation[3]

		qwSq = qw*qw
		x = qx/math.sqrt(1-qwSq)
		y = qy/math.sqrt(1-qwSq)
		z = qz/math.sqrt(1-qwSq)
		return [x, y, z]

	def testOrientation(self):
		currentPose = MoveHelper.get_current_pose()
		orientation = MoveHelper.get_current_orientation()
		axisAngle = self.quaternion_to_axis_angle(orientation)
		print axisAngle

		#orien_euler1 = tf.transformations.euler_from_quaternion(orien)
		#print orien_euler1
		
		rot = tf.transformations.quaternion_from_euler(0, 0, math.pi/2)
		newOrien = tf.transformations.quaternion_multiply(rot, orientation)

		currentPose.orientation = Quaternion(*newOrien)
		orien_euler = tf.transformations.euler_from_quaternion(newOrien)

		MoveHelper.move_to_target_pose(currentPose)
		rospy.sleep(1)
		orientation = MoveHelper.get_current_orientation()
		axisAngle = self.quaternion_to_axis_angle(newOrien)
		print axisAngle
		
	def testGrasp(self, targetPos):
		preGraspPose = MoveHelper.get_current_pose()
		count = 0

		#radAboutZ = math.atan(targetPos[1]targetPos[0])
		# xDiff = targetPos[0]-preGraspPose.position.x
		# yDiff = targetPos[1]-preGraspPose.position.y

		# dist = math.hypot(xDiff, yDiff)

		# while count < 10:

		# 	preGraspPose.position.x += 0.02/dist*xDiff
		# 	preGraspPose.position.y += 0.02/dist*yDiff
		# 	MoveHelper.move_to_target_pose(preGraspPose)
		# 	count += 1

def testLeftArm():
	moveit = KarpalMoveit("left_arm")
	#moveit.test()

	MoveHelper.add_table()
	MoveHelper.add_kinect()

	moveit.group.set_named_target('left_neutral')
	moveit.group.go()

	# rospy.sleep(1)

	milkPos = [0.85, 0.35, -0.072]
	bowlPos = [0.82, 0, -0.07]
	orien = [0.303, 0.711, -0.27, 0.573]
	#moveit.testOrientation()
	picked = moveit.pick(milkPos, fixedOrientation=True)

	# if(picked):
	# 	if (moveit.pour(bowlPos)):
	# 		moveit.place(milkPos)
	"""
	rospy.sleep(1)
	pose = moveit.moveHelper.get_current_pose()
	pose.position.x -= 0.08
	pose.position.y += 0.08
	moveit.move_to_target_pose(pose)

	rospy.sleep(1)
	moveit.group.set_named_target('left_neutral')
	moveit.group.go()

	cerealPos = [0.89, 0.38, -0.055]
	pickOrientation = [0.278, 0.678, -0.19, 0.653]

	picked = moveit.pick(cerealPos)

	if(picked):
		if (moveit.pour(bowlPos)):
			moveit.place(cerealPos)
	"""
def testRightArm():
	moveit = KarpalMoveit("right_arm")
	#moveit.addObject()
	#moveit.group.set_named_target('right_neutral')
	#moveit.group.go()
	moveit.stir(5)
	# Move back to the 'neutral' position (defined in SRDF file)

	"""
	orientation = [0.225, 0.676, -0.217, 0.666]
	if moveit.limb == 'right':
	    rot = tf.transformations.quaternion_from_euler(0, 0, 90*math.pi/180)
	    orientation = tf.transformations.quaternion_multiply(rot, orientation)

	mixerPos = [0.722, -0.421, 0.11]
	pose = moveit.moveHelper.get_current_pose()
	pose.position = Point(*mixerPos)
	pose.orientation = Quaternion(*orientation)
	moveit.move_to_target_pose(pose)
	"""

	# moveit.addObject()
	# moveit.group.set_named_target('right_neutral')
	# moveit.group.go()

	rospy.sleep(2)

	mixerPos = [0.79, -0.40, 0.12]
	pickOrientation = [-0.178, 0.686, 0.157, 0.687]
	bowlPos = [0.84, -0.05, -0.07]
	#moveit.test()
	"""
	picked = moveit.pick(mixerPos, pickOrientation, preGraspDistance = 0.08, fixedOrientation = True)
	if(picked):
		if moveit.move_above_object(bowlPos, moveUp = 0.22, aboveObject=0.20):
			moveit.stir(5)
			moveit.place(mixerPos, moveUp = 0.24, releaseHeight = 0.1)
			#moveit.pour()
			#moveit.place(objectPosition)
	"""
def testBothArm():
	moveitLeft = KarpalMoveit("left_arm")
	moveitLeft.addObject()

	moveitLeft.group.set_named_target('left_neutral')
	moveitLeft.group.go()
	rospy.sleep(1)


	moveitRight = KarpalMoveit("right_arm")
	moveitRight.group.set_named_target('right_neutral')
	moveitRight.group.go()
	rospy.sleep(1)

	milkPos = [0.72, 0.20, -0.072]
	bowlPos = [0.82, -0.05, -0.07]
	cerealPos = [0.86, 0.35, -0.055]
	mixerPos = [0.79, -0.40, 0.12]

	picked = moveitLeft.pick(milkPos, preGraspDistance = 0.08, fixedOrientation = False)

	if(picked):
		print("picked up milk cup")
		if (moveitLeft.pour(bowlPos)):
			print("poured milk to the bowl")
			moveitLeft.place(milkPos)
			print("placed milk cup to original locaion")
	rospy.sleep(3)
	pose = moveitLeft.moveHelper.get_current_pose()
	pose.position.x -= 0.08
	pose.position.y += 0.15
	moveitLeft.move_to_target_pose(pose)

	rospy.sleep(1)
	moveitLeft.group.set_named_target('left_neutral')
	moveitLeft.group.go()
	rospy.sleep(1)

	pickCerealOrien = [0.278, 0.678, -0.19, 0.653]
	picked = moveitLeft.pick(cerealPos)

	if(picked):
		print("picked up cereal box")
	 	if (moveitLeft.pour(bowlPos)):
	 		print("poured cereal to the bowl")
	 		moveitLeft.place(cerealPos)
	 		print("placed cereal box to original locaion")

	pickMixerOrien = [-0.178, 0.686, 0.157, 0.687]
	picked = moveitRight.pick(mixerPos, pickMixerOrien, preGraspDistance = 0.08, fixedOrientation = True)
	if(picked):
		print("picked up mixer")
	 	if moveitRight.move_above_object(bowlPos, moveUp = 0.22, aboveObject=0.20):
	 		print("moved aboved bowl, ready to mix")
	 		moveitRight.stir(radius=0.05, cycles=5)
	 		moveitRight.place(mixerPos, moveUp = 0.24, releaseHeight = 0.1)
			print("placed mixer to original locaion")

if __name__ == "__main__":
	testLeftArm()
	#testRightArm()
	#testBothArm()
	#moveit = KarpalMoveit("left_arm")
	#moveit.test()

	# Shut down MoveIt cleanly
	moveit_commander.roscpp_shutdown()

	# Exit MoveIt
	moveit_commander.os._exit(0)
