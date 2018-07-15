cmake_minimum_required(VERSION 2.8.3)
project(cloud_exercise)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs geometry_msgs eigen_conversions
)

find_package(PCL REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs geometry_msgs eigen_conversions
)

add_executable(cloud_creator src/cloud_creator.cpp)
target_link_libraries(cloud_creator ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(cloud_matcher src/cloud_matcher.cpp)
target_link_libraries(cloud_matcher ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(cloud_loader src/cloud_loader.cpp)
target_link_libraries(cloud_loader ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(cloud_downsampler src/cloud_downsampler.cpp)
target_link_libraries(cloud_downsampler ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(cloud_filter src/cloud_filter.cpp)
target_link_libraries(cloud_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(cloud_planar_segmenter src/cloud_planar_segmenter.cpp)
target_link_libraries(cloud_planar_segmenter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(cloud_clusterer src/cloud_clusterer.cpp)
target_link_libraries(cloud_clusterer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#############
## Install ##
#############
foreach(dir data launch rvizconfig)
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
