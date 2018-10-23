cmake_minimum_required(VERSION 2.8.3)
project(smach_sample)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  smach
  smach_ros
  smach_msgs
)

catkin_package(
 LIBRARIES ${PROJECT_NAME}
 INCLUDE_DIRS 
 CATKIN_DEPENDS roscpp rospy
 DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(simple_gripper src/simple_gripper.cpp)
target_link_libraries(simple_gripper ${catkin_LIBRARIES})

# Install
foreach(dir launch script)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Test
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
