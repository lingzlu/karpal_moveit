import baxter_interface
import moveit_commander
from tf import TransformListener
from geometry_msgs.msg import Pose, PoseStamped
import sys
import math

class MoveHelper(object):
	"""
	Helper class for some common used functions such as get current pose, joint angles, etc.
	make sure call set_arm() to set correct arm
	"""
	limb = None
	arm = None
	group = None

	@classmethod
	def set_arm(cls,limb):
		cls.limb = limb
		cls.arm = baxter_interface.limb.Limb(limb)
		cls.group  = moveit_commander.MoveGroupCommander(limb + '_arm')

	@staticmethod
	def enable_baxter():
		robotEnable = baxter_interface.RobotEnable()
		robotEnable.enable()

	@classmethod
	def move_to_neutral(cls,use_moveit = False):
		if use_moveit:
			return cls.moveit_move_to_neutral(limb)

		cls.arm.move_to_neutral(cls)

	@classmethod
	def moveit_move_to_neutral(cls):
		group  = moveit_commander.MoveGroupCommander(cls.limb + '_arm')
		cls.group.set_named_target(cls.limb + "_neutral")
		cls.group.plan()
		cls.group.go()

	@classmethod
	def move_to_joint_positions(cls,angles):
		cls.arm.move_to_joint_positions(angles)

	@classmethod
	def get_current_joint_values(cls):
		return cls.arm.joint_angles()

	@classmethod
	def get_current_pose(cls):
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

	@classmethod
	def get_current_orientation(cls):
		pose = cls.arm.endpoint_pose()
		return pose['orientation']

	@classmethod
	def move_to_target_pose(cls, targetPose):
		"""
		Move end-effector to a defined pose
		@param targetPose The target pose for the end-effector
		"""
		cls.group.set_start_state_to_current_state()
		cls.group.set_pose_target(targetPose)
		plan = cls.group.plan()
		return cls.group.execute(plan)

	@staticmethod
	def add_table(position = None):
		scene = moveit_commander.PlanningSceneInterface()

		p = PoseStamped()
		p.header.frame_id = "/base"
		if position == None:
			p.pose.position.x = 0.95
			p.pose.position.y = 0
			p.pose.position.z = -0.26
		else:
			p.pose.position.x = position[0]
			p.pose.position.y = position[1]
			p.pose.position.z = position[2]

		scene.add_box("table", p, (1.2, 1.2, 0.2))

	@staticmethod
	def add_kinect(position = None):
		scene = moveit_commander.PlanningSceneInterface()

		p = PoseStamped()
		p.header.frame_id = "/base"

		# add camera
		p.pose.position.x = 0.18
		p.pose.position.y = 0
		p.pose.position.z = 0.54
		scene.add_box("kinect", p, (0.06, 0.2, 0.06))
