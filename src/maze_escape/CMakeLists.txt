cmake_minimum_required(VERSION 2.8.3)
project(maze_escape)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
  CATKIN_DEPENDS rospy
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# catkin_install_python(PROGRAMS src/maze_escape.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

