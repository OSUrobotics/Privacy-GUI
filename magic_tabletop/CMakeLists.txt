cmake_minimum_required(VERSION 2.8.3)
project(magic_tabletop)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(PCL REQUIRED)

find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  message_generation
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
#  DIRECTORY srv
  FILES
  FitPlane.srv
  PublishPlaneTF.srv
  TransformPointCloud2.srv
  #FilterCloudByAxes.srv
  FillInPlane.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES magic_tabletop
#  CATKIN_DEPENDS 
#  roscpp 
#  rospy 
#  std_msgs
#  sensor_msgs
#  geometry_msgs 
#  pcl_conversions
#  pcl_ros
#  message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

#include_directories(SYSTEM
#      ${PCL_COMMON_INCLUDE_DIRS})

## Declare a cpp library
# add_library(magic_tabletop
#   src/${PROJECT_NAME}/magic_tabletop.cpp
# )

## Declare a cpp executable
add_executable(fit_plane src/fit_plane.cpp)
target_link_libraries(fit_plane 
  ${PCL_LIBRARIES}  
  ${catkin_LIBRARIES} 
 )
add_dependencies(fit_plane magic_tabletop_gencpp)

## Declare another cpp executable
add_executable(transform_PointCloud2 src/transform_PointCloud2.cpp)
target_link_libraries(transform_PointCloud2 
  ${PCL_LIBRARIES}  
  ${catkin_LIBRARIES} 
 )
add_dependencies(transform_PointCloud2 magic_tabletop_gencpp)

## Declare another cpp executable
add_executable(fill_in_plane src/fill_in_plane.cpp)
target_link_libraries(fill_in_plane 
  ${PCL_LIBRARIES}  
  ${catkin_LIBRARIES} 
 )
add_dependencies(fill_in_plane magic_tabletop_gencpp)

