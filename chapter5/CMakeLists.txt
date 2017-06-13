cmake_minimum_required(VERSION 2.8.3)
project(chapter5)

find_package(catkin REQUIRED)
catkin_package(CATKIN_DEPENDS)

install(FILES
  usb_cam.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
