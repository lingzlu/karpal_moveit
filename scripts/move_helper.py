import baxter_interface
import moveit_commander
from tf import TransformListener
from geometry_msgs.msg import Pose
import sys
import math

class MoveHelper(object):

	limb = 'left'
	arm = baxter_interface.limb.Limb('left')
	group  = moveit_commander.MoveGroupCommander('left_arm')

	def __init__(cls, limb='left'):
		cls.arm = baxter_interface.limb.Limb(limb)
		cls.scene = moveit_commander.PlanningSceneInterface()

	@staticmethod
	def enable_baxter():
		robotEnable = baxter_interface.RobotEnable()
		robotEnable.enable()

	@classmethod
	def move_to_neutral(use_moveit = False):
		if use_moveit:
			return cls.moveit_move_to_neutral(limb)

		cls.arm.move_to_neutral()

	@classmethod
	def moveit_move_to_neutral():
		cls.group.set_named_target(cls.limb + "_neutral")
		cls.group.plan()
		cls.group.go()

	def create_joint_angles(new_angles, base_angles = None):
		if base_angles is None:
			base_angles = cls.arm.joint_angles()

		angles = copy.deepcopy(base_angles)
		for joint, angle in new_angles.iteritems():
			angles[joint] = angle
		return angles

	def move_to_joint_positions(angles):
		cls.arm.move_to_joint_positions(angles)

	def get_current_joint_values():
		return cls.arm.joint_angles()

	def get_current_pose():
		pose = cls.arm.endpoint_pose()

		current_pose = Pose()
		current_pose.position.x = pose['position'].x
		current_pose.position.y = pose['position'].y
		current_pose.position.z = pose['position'].z
		current_pose.orientation.x = pose['orientation'].x
		current_pose.orientation.y = pose['orientation'].y
		current_pose.orientation.z = pose['orientation'].z
		current_pose.orientation.w = pose['orientation'].w
		return current_pose

	def get_current_orientation():
		pose = cls.arm.endpoint_pose()
		return pose['orientation']
