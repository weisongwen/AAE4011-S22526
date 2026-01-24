cmake_minimum_required(VERSION 3.0.2)
project(aae4011_ai_uas)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  message_generation
)

## Python setup
catkin_python_setup()

## Generate messages
# add_message_files(
#   FILES
#   DetectionResult.msg
#   TrajectoryPoint.msg
# )

## Generate services
# add_service_files(
#   FILES
#   DetectObjects.srv
# )

## Generate added messages and services
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

## Catkin package configuration
catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    nav_msgs 
    visualization_msgs
    message_runtime
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/trajectory_visualizer.py
  scripts/object_detection_node.py
  scripts/logistic_regression_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
