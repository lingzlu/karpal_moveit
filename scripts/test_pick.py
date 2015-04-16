#!/usr/bin/env python

import sys
import rospy
import copy, math

from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random

GROUP_NAME_ARM = 'left_arm'
GROUP_NAME_GRIPPER = 'left_hand'

GRIPPER_FRAME = 'left_gripper'

FIXED_FRAME = 'base'

GRIPPER_CLOSED = 0.3
GRIPPER_OPEN = 0.0

GRIPPER_JOINT_NAMES = ['left_gripper, left_gripper_base']

GRIPPER_EFFORT = [1.0]
class TestPick():
    def __init__(self):

        roscpp_initialize(sys.argv)
        rospy.init_node('pick_place', anonymous=True)
      
        scene = PlanningSceneInterface()
        robot = RobotCommander()
       
        arm = MoveGroupCommander('left_arm')
        gripper = MoveGroupCommander('left_hand')
       
        eef = arm.get_end_effector_link()
       
        rospy.sleep(2)
       
        scene.remove_attached_object(GRIPPER_FRAME, "part")

   
        # clean the scene
        scene.remove_world_object("table")
        scene.remove_world_object("part")
   
        #arm.set_named_target("")
        #arm.go()
      
        #gripper.set_named_target("open")
        #gripper.go()
      
        #rospy.sleep(1)
   
        # publish a demo scene
        p = PoseStamped()
        p.header.frame_id = robot.get_planning_frame()
   
        # add a table
        #p.pose.position.x = 0.42
        #p.pose.position.y = -0.2
        #p.pose.position.z = 0.3
        #scene.add_box("table", p, (0.5, 1.5, 0.6))

   
        # add an object to be grasped
        p.pose.position.x = 1
        p.pose.position.y = 0.25
        p.pose.position.z = -0.05
        scene.add_box("part", p, (0.05, 0.05, 0.1))
      
        rospy.sleep(1)
             
        g = Grasp()
        g.id = "test"
        start_pose = PoseStamped()
        start_pose.header.frame_id = FIXED_FRAME
   
        # start the gripper in a neutral pose part way to the target
        start_pose.pose.position.x = 0.781
        start_pose.pose.position.y = 0.316
        start_pose.pose.position.z = -0.03
        start_pose.pose.orientation.x = 0.022
        start_pose.pose.orientation.y = 0.740
        start_pose.pose.orientation.z = -0.042
        start_pose.pose.orientation.w = 0.670
          
        arm.set_pose_target(start_pose)
        arm.go()
       
        rospy.sleep(2)

        # generate a list of grasps
        grasps = self.make_grasps(start_pose)
   
        result = False
        n_attempts = 0
       
        # repeat until will succeed
        while result == False:
            result = arm.pick("part", grasps)      
            n_attempts += 1
            print "Attempts: ", n_attempts, result
          
        rospy.spin()
        roscpp_shutdown()
       
       
    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self, pose):
        t = JointTrajectory()
        t.joint_names = GRIPPER_JOINT_NAMES
        tp = JointTrajectoryPoint()
        tp.positions = [pose for j in t.joint_names]
        tp.effort = GRIPPER_EFFORT
        t.points.append(tp)
        return t
   
    def make_gripper_translation(self, min_dist, desired, axis=1.0):
        g = GripperTranslation()
        g.direction.vector.x = axis
        g.direction.header.frame_id = GRIPPER_FRAME
        g.min_distance = min_dist
        g.desired_distance = desired
        return g

    def make_grasps(self, pose_stamped, mega_angle=False):
        # setup defaults for the grasp
        g = Grasp()
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)
        g.pre_grasp_approach = self.make_gripper_translation(0.05, 0.1)
        g.post_grasp_retreat = self.make_gripper_translation(0.05, 0.1, -1.0)
        g.grasp_pose = pose_stamped
   
        pitch_vals = [0, 0.2, -0.2, 0.4, -0.4]
        #pitch_vals = [0]
       
        yaw_vals = [-0.2, -0.1, 0, 0.1, 0.2]
        #yaw_vals = [0]
       
        if mega_angle:
            pitch_vals += [0.78, -0.78, 0.3, -0.3, 0.5, -0.5, 0.6, -0.6]
   
        # generate list of grasps
        grasps = []
        #for y in [-1.57, -0.78, 0, 0.78, 1.57]:
        for y in yaw_vals:
            for p in pitch_vals:
                q = quaternion_from_euler(0, 1.57-p, y)
                g.grasp_pose.pose.orientation.x = q[0]
                g.grasp_pose.pose.orientation.y = q[1]
                g.grasp_pose.pose.orientation.z = q[2]
                g.grasp_pose.pose.orientation.w = q[3]
                g.id = str(len(grasps))
                g.allowed_touch_objects = ["part"]
                g.max_contact_force = 0
                #g.grasp_quality = 1.0 - abs(p/2.0)
                grasps.append(copy.deepcopy(g))
        return grasps


if __name__=='__main__':
    TestPick()