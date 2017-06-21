cmake_minimum_required(VERSION 2.8.3)
project(tf_tutorial)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tf_tutorial
   CATKIN_DEPENDS roscpp rospy tf2
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(tf2_broadcaster src/tf2_broadcaster.cpp)
target_link_libraries(tf2_broadcaster ${catkin_LIBRARIES})
