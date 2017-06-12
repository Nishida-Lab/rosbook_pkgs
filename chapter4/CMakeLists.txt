cmake_minimum_required(VERSION 2.8.3)
project(chapter4)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
)

###########
## Build ##
###########

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  geometry_msgs
  )

add_definitions(-std=c++11)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rqtplot_example src/rqtplot_example.cpp)
target_link_libraries(rqtplot_example ${catkin_LIBRARIES})

add_executable(rosbag_example_publisher src/rosbag_example_publisher.cpp)
target_link_libraries(rosbag_example_publisher ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS
  rqtplot_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir bag config launch output)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
