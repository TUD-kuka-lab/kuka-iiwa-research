cmake_minimum_required(VERSION 3.0.2)
project(cor_tud_controllers)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs # controller action server 
  std_msgs #TODO: not sure if its needed to build msgs
)

# TODO: should we set this?
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cor_tud_controllers
	CATKIN_DEPENDS cor_tud_msgs
#  DEPENDS system_lib
)

#include_directories(
# include
#  ${catkin_INCLUDE_DIRS}
#)

catkin_install_python(PROGRAMS
  python/add_namespace.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
