#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <iostream>

// For time parametrisation
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cartesian_path_plan");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("right_arm");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  moveit::planning_interface::MoveGroup::Plan plan;
  group.setStartStateToCurrentState();

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose start_pose = group.getCurrentPose().pose;
  geometry_msgs::Pose target_pose = start_pose;
  waypoints.push_back(start_pose);

  target_pose.position.x -= 0.1;
  target_pose.position.y -= 0.05;
  waypoints.push_back(target_pose);

  // target_pose.position.y += 0.01;
  // waypoints.push_back(target_pose);

  // target_pose.position.x -= 0.08;
  // waypoints.push_back(target_pose);

  moveit_msgs::RobotTrajectory trajectory_msg;
  group.setPlanningTime(10.0);

  double fraction = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory_msg,
                                               false);

  ROS_INFO("Cartesian path plan (%.2f%% acheived)",
        fraction * 100.0);

  // reate a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "right_arm");

  // get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_msg);

  // create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  // ompute computeTimeStamps
  bool success = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory_msg);
  // Check trajectory_msg for velocities not empty
 // std::cout << trajectory_msg << std::endl;

  plan.trajectory_ = trajectory_msg;
  ROS_INFO("Cartesian path plan  (%.2f%% acheived)",fraction * 100.0);
  sleep(5.0);

  group.execute(plan);

  ros::shutdown();
  return 0;
}
