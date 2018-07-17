cmake_minimum_required(VERSION 2.8.3)
project(chapter5)

find_package(catkin REQUIRED COMPONENTS
    rospy sensor_msgs std_msgs)

catkin_package(
    CATKIN_DEPENDS rospy sensor_msgs std_msgs)

foreach(dir launch config script)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
