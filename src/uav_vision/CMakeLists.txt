cmake_minimum_required(VERSION 3.0.2)
project(uav_vision)

find_package(catkin REQUIRED COMPONENTS
  rospy
  cv_bridge
  image_transport
  sensor_msgs
  geometry_msgs
  std_msgs
  uav_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS 
    rospy
    cv_bridge
    image_transport
    sensor_msgs
    geometry_msgs
    std_msgs
    uav_msgs
    tf
)

catkin_install_python(PROGRAMS
  nodes/object_detector.py
  nodes/object_tracker.py
  nodes/vision_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

