cmake_minimum_required(VERSION 2.8.3)
project(privacy_zones)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs message_generation)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_python_setup()

add_message_files(
  FILES
  Zone.msg
  Transition.msg
  ZoneControl.msg
)

add_service_files(
  FILES
  DevicesInZone.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  peac_bridge
)


catkin_package()