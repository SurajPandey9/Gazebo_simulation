cmake_minimum_required(VERSION 3.0.2)
project(uav_simulation)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  uav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf
)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
    gazebo_ros 
    gazebo_plugins 
    uav_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)

# Install model files
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

