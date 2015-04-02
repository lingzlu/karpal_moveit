#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

void demoPick(moveit::planning_interface::MoveGroup &group)
{
  std::vector<moveit_msgs::Grasp> grasps;
  for (std::size_t i = 0 ; i < 20 ; ++i)
  {
    geometry_msgs::PoseStamped p = group.getRandomPose();
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    moveit_msgs::Grasp g;
    g.grasp_pose = p;
    g.pre_grasp_approach.direction.vector.x = 1.0;
    g.post_grasp_retreat.direction.vector.z = 1.0;
    g.post_grasp_retreat.direction.header = p.header;
    g.pre_grasp_approach.min_distance = 0.2;
    g.pre_grasp_approach.desired_distance = 0.4;
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.27;
    g.pre_grasp_posture.joint_names.resize(1, "r_gripper_joint");
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(1);
    g.pre_grasp_posture.points[0].positions[0] = 1;

    g.grasp_posture.joint_names.resize(1, "r_gripper_joint");
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.resize(1);
    g.grasp_posture.points[0].positions[0] = 0;

    grasps.push_back(g);
  }
  group.pick("bubu", grasps);
}

void demoPlace(moveit::planning_interface::MoveGroup &group)
{
  std::vector<moveit_msgs::PlaceLocation> loc;
  for (std::size_t i = 0 ; i < 20 ; ++i)
  {
    geometry_msgs::PoseStamped p = group.getRandomPose();
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    moveit_msgs::PlaceLocation g;
    g.place_pose = p;
    g.pre_place_approach.direction.vector.x = 1.0;
    g.post_place_retreat.direction.vector.z = 1.0;
    g.post_place_retreat.direction.header = p.header;
    g.pre_place_approach.min_distance = 0.2;
    g.pre_place_approach.desired_distance = 0.4;
    g.post_place_retreat.min_distance = 0.1;
    g.post_place_retreat.desired_distance = 0.27;

    g.post_place_posture.joint_names.resize(1, "r_gripper_joint");
    g.post_place_posture.points.resize(1);
    g.post_place_posture.points[0].positions.resize(1);
    g.post_place_posture.points[0].positions[0] = 0;

    loc.push_back(g);
  }
  group.place("bubu", loc);
}

void attachObject(void)
{

}
int main(int argc, char **argv)

{

  ros::init(argc, argv, "MoveGroupInterface", ros::init_options::AnonymousName);

  // start a ROS spinning thread

  ros::AsyncSpinner spinner(1);

  spinner.start();

  // this connects to a running instance of the move_group node

  move_group_interface::MoveGroup group("right_arm");

 

 // (Optional) Create a publisher for visualizing plans in Rviz.

  //ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  //moveit_msgs::DisplayTrajectory display_trajectory;

 
 // moveit::planning_interface::MoveGroup group(argc > 1 ? argv[1] : "right_arm");
  demoPlace(group);

  sleep(2);

  return 0;
}
