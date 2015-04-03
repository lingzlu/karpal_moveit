#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>

void pick(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose& object_pose)
{
    std::vector<moveit_msgs::Grasp> possible_grasps;

    moveit_msgs::Grasp grasp;
    grasp.id = "Test Grasp";
    grasp.grasp_quality = 1.0;

    std::vector<std::string> joint_names;
    joint_names.push_back("right_gripper");

    trajectory_msgs::JointTrajectory pre_grasp_posture;
    pre_grasp_posture.header.frame_id = "root";
    pre_grasp_posture.header.stamp = ros::Time::now();
    pre_grasp_posture.joint_names = joint_names;
    pre_grasp_posture.points.resize(1);
    pre_grasp_posture.points[0].positions.push_back(0);
    pre_grasp_posture.points[0].positions.push_back(0);
    pre_grasp_posture.points[0].positions.push_back(0);
    pre_grasp_posture.points[0].positions.push_back(0);
    pre_grasp_posture.points[0].positions.push_back(0);
    pre_grasp_posture.points[0].positions.push_back(0);
    pre_grasp_posture.points[0].time_from_start = ros::Duration(4.0);
    grasp.pre_grasp_posture = pre_grasp_posture;

    trajectory_msgs::JointTrajectory grasp_posture;
    grasp_posture.header.frame_id = "root";
    grasp_posture.header.stamp = ros::Time::now();
    grasp_posture.joint_names = joint_names;
    grasp_posture.points.resize(1);
    grasp_posture.points[0].positions.push_back(0.15);
    grasp_posture.points[0].positions.push_back(0.15);
    grasp_posture.points[0].positions.push_back(0.15);
    grasp_posture.points[0].positions.push_back(0.6981);
    grasp_posture.points[0].positions.push_back(0.6981);
    grasp_posture.points[0].positions.push_back(0.6981);
    grasp_posture.points[0].time_from_start = ros::Duration(4.0);
    grasp.grasp_posture = grasp_posture;

    geometry_msgs::PoseStamped grasp_pose_msg;
    grasp_pose_msg.header.stamp = ros::Time::now();
    grasp_pose_msg.header.frame_id = "/root";
    grasp_pose_msg.pose = object_pose;
    grasp_pose_msg.pose.position.y -= 0.1525;

    Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(M_PI/2, Eigen::Vector3d::UnitX());
    Eigen::Quaternion<double> quat = rollAngle * yawAngle * pitchAngle;
    grasp_pose_msg.pose.orientation.x = quat.x();
    grasp_pose_msg.pose.orientation.y = quat.y();
    grasp_pose_msg.pose.orientation.z = quat.z();
    grasp_pose_msg.pose.orientation.w = quat.w();
    grasp.grasp_pose = grasp_pose_msg;

    // grasp.grasp_pose = p;
    grasp.pre_grasp_approach.direction.vector.x = 1.0;
    grasp.pre_grasp_approach.min_distance = 0.2;
    grasp.pre_grasp_approach.desired_distance = 0.4;

    grasp.post_grasp_retreat.direction.vector.z = 1.0;
    grasp.post_grasp_retreat.direction.header = p.header;
    grasp.post_grasp_retreat.min_distance = 0.1;
    grasp.post_grasp_retreat.desired_distance = 0.27;

    // grasp.grasp_posture.joint_names = joint_names;
    // grasp.grasp_posture.points.resize(1);
    // grasp.grasp_posture.points[0].positions.resize(1);
    // grasp.grasp_posture.points[0].positions[0] = 0;

    possible_grasps.push_back(grasp);
    group.pick("block", possible_grasps);
}

void place(moveit::planning_interface::MoveGroup &group)
{
  std::vector<moveit_msgs::PlaceLocation> loc;
  for (std::size_t i = 0 ; i < 10 ; ++i)
  {
    geometry_msgs::PoseStamped p = group.getRandomPose();
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    moveit_msgs::PlaceLocation grasp;
    grasp.place_pose = p;
    grasp.pre_place_approach.direction.vector.x = 1.0;
    grasp.post_place_retreat.direction.vector.z = 1.0;
    grasp.post_place_retreat.direction.header = p.header;
    grasp.pre_place_approach.min_distance = 0.2;
    grasp.pre_place_approach.desired_distance = 0.4;
    grasp.post_place_retreat.min_distance = 0.1;
    grasp.post_place_retreat.desired_distance = 0.27;

    grasp.post_place_posture.joint_names.resize(1, "right_gripper");
    grasp.post_place_posture.points.resize(1);
    grasp.post_place_posture.points[0].positions.resize(1);
    grasp.post_place_posture.points[0].positions[0] = 0;

    loc.push_back(grasp);
  }
  group.place("block", loc);
}

void generateTestObject(geometry_msgs::Pose& object_pose)
{
  // Position
  geometry_msgs::Pose start_object_pose;
  start_object_pose.position.x = 0.3;
  start_object_pose.position.y = 0.5;
  start_object_pose.position.z =  0.25;

  // Orientation
  double angle = 0; // M_PI / 1.5;
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
  start_object_pose.orientation.x = quat.x();
  start_object_pose.orientation.y = quat.y();
  start_object_pose.orientation.z = quat.z();
  start_object_pose.orientation.w = quat.w();

  // Choose which object to test
  object_pose = start_object_pose;

  // Show the block
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
  visual_tools->cleanupACO("Cylinder");
  visual_tools->publishCollisionFloor(0.0, "Floor", rviz_visual_tools::BLUE);
  visual_tools->publishCollisionTable(0.0, 0.75, 0.0, 0.8, 0.18, 1.5, "Table", rviz_visual_tools::BLUE);
  std::ostringstream stringStream;
  stringStream << "Cylinder " ;
  std::string object_name = stringStream.str();
  visual_tools->publishCollisionCylinder(object_pose, object_name, 0.035, 0.25);
  ros::Duration(0.5).sleep();
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "MoveGroupInterface", ros::init_options::AnonymousName);

  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group(argc > 1 ? argv[1] : "right_arm");

  geometry_msgs::Pose object_pose;
  generateTestObject(object_pose);

  ros::WallDuration(1.0).sleep();
  pick(group, object_pose);

  ros::WallDuration(1.0).sleep();

  place(group);

  ros::waitForShutdown();

  return 0;
}
