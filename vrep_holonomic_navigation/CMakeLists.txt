cmake_minimum_required(VERSION 2.8.3)
project(vrep_holonomic_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  sensor_msgs
  laser_geometry
)

### BUild

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

### Install
catkin_install_python(PROGRAMS
  scripts/laserscan_to_pointcloud.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
