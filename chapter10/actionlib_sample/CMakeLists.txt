cmake_minimum_required(VERSION 2.8.3)
project(actionlib_sample)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  actionlib_tutorials
  pr2_controllers_msgs
  std_msgs
  roscpp
  rospy
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
   actionlib
   actionlib_msgs
   pr2_controllers_msgs
   std_msgs
   roscpp
)

