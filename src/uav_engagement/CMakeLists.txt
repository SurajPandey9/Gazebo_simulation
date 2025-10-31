cmake_minimum_required(VERSION 3.0.2)
project(uav_engagement)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  uav_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    rospy
    std_msgs
    geometry_msgs
    uav_msgs
)

catkin_install_python(PROGRAMS
  nodes/engagement_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

