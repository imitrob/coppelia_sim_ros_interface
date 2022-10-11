cmake_minimum_required(VERSION 3.0.2)
project(coppelia_sim_ros_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_visual_tools
  message_generation
)

add_message_files(
  FILES
  ObjectInfo.msg
  EEPoseGoals.msg
  JointAngles.msg
)

add_service_files (
  FILES
  AddOrEditObject.srv
  RemoveObject.srv
  GripperControl.srv
  AddLine.srv
)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
