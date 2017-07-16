cmake_minimum_required(VERSION 2.8.3)
project(smach_sample)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  smach_ros
  smach_msgs
)

catkin_package(
 CATKIN_DEPENDS rospy
)

