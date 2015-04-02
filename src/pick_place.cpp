#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>

void pick(moveit::planning_interface::MoveGroup &group)
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

    // grasp.pre_grasp_posture.joint_names = joint_names;
    // grasp.pre_grasp_posture.points.resize(1);
    // grasp.pre_grasp_posture.points[0].positions.resize(1);
    // grasp.pre_grasp_posture.points[0].positions[0] = 1;

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

    // geometry_msgs::PoseStamped p = group.getRandomPose();
    // p.pose.orientation.x = 0;
    // p.pose.orientation.y = 0;
    // p.pose.orientation.z = 0;
    // p.pose.orientation.w = 1;
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

void attachObject(void)
{
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<moveit_msgs::CollisionObject>("/collision_object", 1000);
  sleep(1);

  ros::Rate rate(10);
  moveit_msgs::CollisionObject msg;
  msg.header.frame_id = "/base_link";
  msg.id = "block";
  msg.primitives.resize(1);
  msg.primitives[0].type = msg.primitives[0].CYLINDER;
  msg.primitives[0].dimensions.push_back(0.1);//height
  msg.primitives[0].dimensions.push_back(0.05);//radius
  msg.primitive_poses.resize(1);
  msg.primitive_poses[0].position.x = 0.8;
  msg.primitive_poses[0].position.y = 0.2;
  msg.primitive_poses[0].position.z = 0.3;
  msg.primitive_poses[0].orientation.x = 0.0;
  msg.primitive_poses[0].orientation.y = 0.0;
  msg.primitive_poses[0].orientation.z = 0.0;
  msg.primitive_poses[0].orientation.w = 1.0;
  msg.operation = msg.ADD;

  pub.publish(msg);

}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "MoveGroupInterface", ros::init_options::AnonymousName);

  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  spinner.start();
  attachObject();
  // ros::NodeHandle nh;
  // ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  // ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

  // ros::WallDuration(1.0).sleep();

  // this connects to a running instance of the move_group node
  moveit::planning_interface::MoveGroup group(argc > 1 ? argv[1] : "right_arm");
  group.setPlanningTime(45.0);

 // (Optional) Create a publisher for visualizing plans in Rviz.
  //ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  //moveit_msgs::DisplayTrajectory display_trajectory;

  // moveit_msgs::CollisionObject co;
  // co.header.stamp = ros::Time::now();
  // co.header.frame_id = "base_footprint";

  //  // remove pole
  // co.id = "pole";
  // co.operation = moveit_msgs::CollisionObject::REMOVE;
  // pub_co.publish(co);
  // // add pole
  // co.operation = moveit_msgs::CollisionObject::ADD;
  // co.primitives.resize(1);
  // co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  // co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  // co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
  // co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  // co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
  // co.primitive_poses.resize(1);
  // co.primitive_poses[0].position.x = 0.7;
  // co.primitive_poses[0].position.y = -0.4;
  // co.primitive_poses[0].position.z = 0.85;
  // co.primitive_poses[0].orientation.w = 1.0;
  // pub_co.publish(co);

  // remove table
  // co.id = "table";
  // co.operation = moveit_msgs::CollisionObject::REMOVE;
  // pub_co.publish(co);
  // // add table
  // co.operation = moveit_msgs::CollisionObject::ADD;
  // co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
  // co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
  // co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
  // co.primitive_poses[0].position.x = 0.7;
  // co.primitive_poses[0].position.y = -0.2;
  // co.primitive_poses[0].position.z = 0.175;
  // pub_co.publish(co);

  // co.id = "block";
  // co.operation = moveit_msgs::CollisionObject::REMOVE;
  // pub_co.publish(co);

  // moveit_msgs::AttachedCollisionObject aco;
  // aco.object = co;
  // pub_aco.publish(aco);

  // co.operation = moveit_msgs::CollisionObject::ADD;
  // // co.primitives.resize(1);
  // // co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  // co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  // co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.64;
  // co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.4;
  // co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
  // co.primitive_poses[0].position.x = 0.6;
  // co.primitive_poses[0].position.y = -0.7;
  // co.primitive_poses[0].position.z = 0.5;
  // pub_co.publish(co);

  ros::WallDuration(1.0).sleep();
  pick(group);

  ros::WallDuration(1.0).sleep();

  place(group);

  ros::waitForShutdown();

  return 0;
}
