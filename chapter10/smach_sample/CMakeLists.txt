cmake_minimum_required(VERSION 2.8.3)
project(smach_sample)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  std_msgs
  pr2_controllers_msgs
  roscpp
  rospy
)

catkin_package(
 LIBRARIES ${PROJECT_NAME}
 INCLUDE_DIRS 
 CATKIN_DEPENDS
   actionlib
   actionlib_msgs
   pr2_controllers_msgs
   std_msgs
   roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

add_executable(simple_gripper src/simple_gripper.cpp)
target_link_libraries(simple_gripper ${catkin_LIBRARIES})

###########
# Install #
###########

set(EXEC_FILES
  simple_gripper
)

# Install executables
foreach(nodes ${EXEC_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

# Python scripts
set(PYTHON_SCRIPTS
  script/smach_pr2.py
  script/smach_sample.py
)

foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

foreach(dir launch script)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
