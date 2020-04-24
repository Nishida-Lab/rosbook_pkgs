cmake_minimum_required(VERSION 2.8.3)
project(actionlib_sample)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

#############
## Install ##
#############

# Python scripts
set(PYTHON_SCRIPTS
  script/simple_client.py
  script/simple_server.py
  script/simple_server2.py
  script/ten_times_publisher.py
)

foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()
