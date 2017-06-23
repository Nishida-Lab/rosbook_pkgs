cmake_minimum_required(VERSION 2.8.3)
project(arrayutil_python)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
 CATKIN_DEPENDS rospy
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest)
  add_rostest(test/arrayutil_python.test)
endif()
