#!/usr/bin/env python

import rospy, sys
import moveit_commander
import math
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy

class Path_Planning:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)

        # Connect to the right_arm move group
        right_arm = MoveGroupCommander('right_arm')

        # Allow replanning to increase the odds of a solution
        right_arm.allow_replanning(True)

        # Set the right arm reference frame
        #right_arm.set_pose_reference_frame('base_footprint')

        # Allow some leeway in position(meters) and orientation (radians)
        right_arm.set_goal_position_tolerance(0.001)
        right_arm.set_goal_orientation_tolerance(0.01)

        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()

         # Start in the "straight_forward" configuration stored in the SRDF file
        right_arm.set_named_target('right_neutral')

        # Plan and execute a trajectory to the goal configuration
        right_arm.go()
        rospy.sleep(1)
        # Get the current pose so we can add it as a waypoint
        start_pose = right_arm.get_current_pose(end_effector_link).pose

        waypoints = self.circular_path(start_pose)
        print waypoints
        fraction = 0.0
        maxtries = 100
        attempts = 0

        # Set the internal state to the current state
        right_arm.set_start_state_to_current_state()

        # Plan the Cartesian path connecting the waypoints
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = right_arm.compute_cartesian_path (
                                     waypoints,   # waypoint poses
                                     0.01,        # eef_step
                                     0.0,         # jump_threshold
                                     False)        # avoid_collisions

            # Increment the number of attempts
            attempts += 1

            # Print out a progress message
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

         # If we have a complete plan, execute the trajectory
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")

            right_arm.execute(plan)

            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")

            # Move normally back to the 'resting' position
            right_arm.set_named_target('right_neutral')
            right_arm.go()
            rospy.sleep(1)

            # Shut down MoveIt cleanly
            moveit_commander.roscpp_shutdown()

            # Exit MoveIt
            moveit_commander.os._exit(0)

    def circular_path(self, current_pose):

        waypoints = []

        # Append the pose to the waypoints list
        wpose = deepcopy(current_pose)
        waypoints.append(wpose)
        radius = 0.05
        for a in range(0, 10, 1):
            tempX = radius*math.cos(a*math.pi/4)
            tempY = radius*math.sin(a*math.pi/4)
            wpose.position.x += tempX
            wpose.position.y += tempY
            waypoints.append(deepcopy(wpose))

        # Append the start pose to the waypoints list
        waypoints.append(current_pose)
        return waypoints


if __name__ == "__main__":
     try:
         Path_Planning()
     except rospy.ROSInterruptException:
         pass
