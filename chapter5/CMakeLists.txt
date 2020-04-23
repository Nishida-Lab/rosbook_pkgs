cmake_minimum_required(VERSION 2.8.3)
project(chapter5)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    sensor_msgs
    std_msgs
)

#############
## Install ##
#############

# Python scripts
set(PYTHON_SCRIPTS
  script/joy_pan_tilt.py
)

foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

foreach(dir launch config script)
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
