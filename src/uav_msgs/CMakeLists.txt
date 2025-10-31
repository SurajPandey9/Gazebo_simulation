cmake_minimum_required(VERSION 3.0.2)
project(uav_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Detection.msg
  DetectionArray.msg
  TargetState.msg
  MissionStatus.msg
  UAVState.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  EngageTarget.srv
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs
)

