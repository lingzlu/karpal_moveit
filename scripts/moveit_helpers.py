import baxter_interface
import moveit_commander
from tf import TransformListener
import sys

class MoveHelper:
	
	def __init__(self, limb='left'):
		self.arm = baxter_interface.limb.Limb(limb)
		
	def move_to_neutral(self):
		self.arm.move_to_neutral()
		
	def create_joint_angles(self, new_angles, base_angles = None):
		if base_angles is None:
			base_angles = self.arm.joint_angles()

		angles = copy.deepcopy(base_angles)
		for joint, angle in new_angles.iteritems():
			angles[joint] = angle
		return angles

	def get_current_joint_values(self):
		return self.arm.joint_angles()

	def get_current_pose(self):
		return self.arm.endpoint_pose()
	
