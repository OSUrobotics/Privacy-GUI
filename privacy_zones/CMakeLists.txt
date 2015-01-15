cmake_minimum_required(VERSION 2.8.3)
project(privacy_zones)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rospy)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_python_setup()


catkin_package()