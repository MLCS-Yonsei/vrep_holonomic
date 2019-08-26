cmake_minimum_required(VERSION 2.8.3)
project(vrep_holonomic_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  joy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roscpp rospy geometry_msgs joy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(vrep_holonomic_teleop_joy src/vrep_holonomic_teleop_joy.cpp)
target_link_libraries(vrep_holonomic_teleop_joy ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/vrep_holonomic_teleop_key
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

