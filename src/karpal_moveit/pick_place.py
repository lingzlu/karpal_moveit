#!/usr/bin/env python
import rospy, sys
sys.path.append('/home/course/lu3421/catkin_ws/src/gripping/src')
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
import moveit_msgs.msg

from grasp import Gripper

class Pick_Place:

	def __init__(self, planArm="left_arm"):
		# Initialize the move_group API
		#moveit_commander.roscpp_initialize(sys.argv)

		# Initialize the ROS node
		rospy.init_node('pick_place', anonymous=True)

		# Connect to the arm move group
		if planArm == "right_arm":
			self.arm = MoveGroupCommander('right_arm')
			self.hand = Gripper(1)
		else:
			self.arm = MoveGroupCommander('left_arm')
			self.hand = Gripper(0)

		rospy.sleep(1)
		self.hand.release()
		
		# Allow replanning to increase the odds of a solution
		self.arm.allow_replanning(True)

		# Allow some leeway in position(meters) and orientation (radians)
		#self.arm.set_goal_position_tolerance(0.001)
		#self.arm.set_goal_orientation_tolerance(0.01)

	def pick(self):
		self.graspFromSide()
	
	def graspFromSide(self):

		targetPose=Pose()
		targetPose.position.x = 0.96
		targetPose.position.y = 0.10
		targetPose.position.z = -0.0

		targetPose.orientation.x = 0.044
		targetPose.orientation.y = 0.702
		targetPose.orientation.z = 0.045
		targetPose.orientation.w = 0.708

		# right side grasp
		# Position
		#x = 0.636
		#y = 0.832
		#z = 0.192

		# Orientation
		#x = -0.382
		#y = 0.922
		#z = 0.021
		#w= 0.049
		
		#xyz 0.871 0.428 0.038 
		#orientation 0.119 0.698 -0.083 0.700

		self.moveToTargetPose(targetPose)

		while self.hand.range.state() == 65535.0:
			targetPose.position.x += 0.05
			self.moveToTargetPose(targetPose)
		
		while self.hand.range.state() > 70:
			targetPose.position.x += 0.02
			self.moveToTargetPose(targetPose)
		
		self.hand.grasp()
		rospy.sleep(1)
		
		targetPose.position.x -= 0.05
		targetPose.position.y -= 0.05
		targetPose.position.z += 0.1
		self.moveToTargetPose(targetPose)

		self.hand.release()
		rospy.sleep(1)
		
	def moveToTargetPose(self, targetPose):
		# move to the defined pose
		self.arm.set_start_state_to_current_state()
		self.arm.set_pose_target(targetPose)
		self.arm.go()
		rospy.sleep(0.5)	

	#def graspFromTop(self):

	

if __name__ == "__main__":
	
	# demo: do circular motion on left arm
	#rospy.init_node('test',anonymous=True)
	#hand = Gripper(0)
	#hand.release()
	#hand.grasp()
	pick_place = Pick_Place("left_arm")
	rospy.sleep(1)

	# Move back to the 'neutral' position (defined in SRDF file)
	#pick_place.arm.set_named_target('left_neutral')
	#pick_place.arm.go()
	#rospy.sleep(1)

	pick_place.pick()
	
	# Shut down MoveIt cleanly
	#moveit_commander.roscpp_shutdown()

	# Exit MoveIt
	#moveit_commander.os._exit(0)
