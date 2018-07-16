cmake_minimum_required(VERSION 2.8.3)
project(fourth_robot_gazebo)
find_package(catkin REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS fourth_robot_description
)

#############
## Install ##
#############
foreach(dir launch config worlds) 
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
