cmake_minimum_required(VERSION 2.8.3)
project(actionlib_sample)

find_package(catkin REQUIRED COMPONENTS
  actionlib actionlib_msgs actionlib_tutorials std_msgs rospy
)

catkin_package(
 CATKIN_DEPENDS actionlib_msgs actionlib_tutorials std_msgs rospy
)

