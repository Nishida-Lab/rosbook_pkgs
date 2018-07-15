cmake_minimum_required(VERSION 2.8.3)
project(chapter5)

find_package(catkin REQUIRED)
catkin_package(CATKIN_DEPENDS)

set(LAUNCH_DIRS launch)

foreach(dir ${LAUNCH_DIRS})
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
