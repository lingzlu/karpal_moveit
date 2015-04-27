import baxter_interface
import moveit_commander
from tf import TransformListener
from geometry_msgs.msg import Pose
import sys
import math
import numpy as np

class MoveHelper:
	
	def __init__(self, limb='left'):
		self.arm = baxter_interface.limb.Limb(limb)
		
	def enableBaxterInterface(self):

		robotEnable = baxter_interface.RobotEnable()
		robotEnable.enable()

	def move_to_neutral(self):
		self.arm.move_to_neutral()
		
	def create_joint_angles(self, new_angles, base_angles = None):
		if base_angles is None:
			base_angles = self.arm.joint_angles()

		angles = copy.deepcopy(base_angles)
		for joint, angle in new_angles.iteritems():
			angles[joint] = angle
		return angles
	
	def move_to_joint_positions(self, angles):
		self.arm.move_to_joint_positions(angles)

	def get_current_joint_values(self):
		return self.arm.joint_angles()

	def get_current_pose(self):
		pose = self.arm.endpoint_pose()

		current_pose = Pose()
		current_pose.position.x = pose['position'].x
		current_pose.position.y = pose['position'].y
		current_pose.position.z = pose['position'].z
		current_pose.orientation.x = pose['orientation'].x
		current_pose.orientation.y = pose['orientation'].y
		current_pose.orientation.z = pose['orientation'].z
		current_pose.orientation.w = pose['orientation'].w
		return current_pose
	
	def quaternion_multiply(self, q1, q2):
		
   		x1, y1, z1, w1 = q1
	    	x2, y2, z2, w2 = q2

	    	return np.array([x1*w2 + y1*z2 - z1*y2 + w1*x2,
	                        -x1*z2 + y1*w2 + z1*x2 + w1*y2,
	                         x1*y2 - y1*x2 + z1*w2 + w1*z2,
				-x1*x2 - y1*y2 - z1*z2 + w1*w2], dtype=np.float64)
	"""
	def get_current_pose(self):
		end_effector_link = self.group.get_end_effector_link()
		self.tfl.waitForTransform("base", end_effector_link, rospy.Time(0), rospy.Duration(4.0))
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
	"""
	
	def get_current_orientation(self):
		pose = self.arm.endpoint_pose()
		return pose['orientation']

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
