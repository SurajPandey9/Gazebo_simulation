cmake_minimum_required(VERSION 3.0.2)
project(uav_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  uav_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS 
    rospy
    geometry_msgs
    nav_msgs
    std_msgs
    sensor_msgs
    uav_msgs
    tf
)

catkin_install_python(PROGRAMS
  nodes/flight_controller.py
  nodes/mission_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

