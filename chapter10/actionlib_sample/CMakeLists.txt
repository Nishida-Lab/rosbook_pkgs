cmake_minimum_required(VERSION 2.8.3)
project(actionlib_sample)

find_package(catkin REQUIRED COMPONENTS
  rospy actionlib actionlib_msgs actionlib_tutorials
)

catkin_package(
 CATKIN_DEPENDS rospy actionlib_msgs actionlib_tutorials
)

