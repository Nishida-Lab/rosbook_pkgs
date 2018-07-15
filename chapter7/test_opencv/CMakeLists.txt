cmake_minimum_required(VERSION 2.8.3)
project(test_opencv)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(test_opencv src/test_opencv.cpp)
target_link_libraries(test_opencv ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(opencv_ros src/opencv_ros.cpp)
target_link_libraries(opencv_ros ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(test_get_input src/test_get_input.cpp)
target_link_libraries(test_get_input ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(test_edge_detection src/test_edge_detection.cpp)
target_link_libraries(test_edge_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(test_feature_detection src/test_feature_detection.cpp)
target_link_libraries(test_feature_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(test_feature_detection_matching src/test_feature_detection_matching.cpp)
target_link_libraries(test_feature_detection_matching ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(test_svm src/test_svm.cpp)
target_link_libraries(test_svm ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

