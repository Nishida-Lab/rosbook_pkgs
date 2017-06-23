cmake_minimum_required(VERSION 2.8.3)
project(pluginlib_arrayutil_client)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  pluginlib_arrayutil
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS pluginlib roscpp
)

include_directories(
  include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(client_node src/client_node.cpp src/pluginlib_arrayutil_client.cpp)
target_link_libraries(client_node ${catkin_LIBRARIES})

