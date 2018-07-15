cmake_minimum_required(VERSION 2.8.3)
project(fourth_robot_control)
find_package(catkin REQUIRED)

#############
## Install ##
#############
foreach(dir launch config)
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
