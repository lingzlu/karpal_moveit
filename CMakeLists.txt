cmake_minimum_required(VERSION 2.8.3)
project(karpal_moveit)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  eigen_conversions
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_core
  tf
  tf_conversions
  trajectory_msgs
  message_generation
  geometry_msgs
  actionlib
  actionlib_msgs
  moveit_msgs
  cmake_modules
)

find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED thread system)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    moveit_msgs
    trajectory_msgs
    std_msgs
    moveit_visual_tools
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(pick_place src/pick_place.cpp)
#add_executable(cartesian_path src/circular_path.cpp)

#target_link_libraries(cartesian_path
#  ${catkin_LIBRARIES}
#)
