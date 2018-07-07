cmake_minimum_required(VERSION 2.8.3)
project(chapter7)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(test_opencv src/test_opencv.cpp)
target_link_libraries(test_opencv ${OpenCV_LIBRARIES})

