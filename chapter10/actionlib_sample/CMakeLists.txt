cmake_minimum_required(VERSION 2.8.3)
project(actionlib_sample)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
)

