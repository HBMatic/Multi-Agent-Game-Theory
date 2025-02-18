cmake_minimum_required(VERSION 3.0.2)
project(multi-agent-game-theory)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    geometry_msgs
    nav_msgs
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS
    odometry_analysis/move_circle.py
    odometry_analysis/move_circle_plot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
