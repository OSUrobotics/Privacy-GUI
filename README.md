Privacy Interfaces GUI
==================

Welcome to the Privacy Interfaces repo!  Please check out the Wiki for current information and project goals. 
This repo is dedicated to the multiple graphic user interfaces needed for the privacy project. If you are looking for non-graphic related privacy packages, please check out OSUrobotics/privacy-interfaces

Authors: 
* Suzanne (Interface layout and design)
* Alex (Localization and navigation)
* Penn (Filters and Image Products)

##Why Groovy?
* The current version of the rviz library used in python does not support hydro
* Mixing groovy and hydro causes crashes and segfaults
* If you are running any of these packages, **have a separate groovy workspace**

##Package list and info
* remote_nav - The real package and current implementation of our navigation UI
* map_registration - not a ROS package but a pyQT program which is used for map registration

##custom_nav package
The custom_nav package is to be used ON THE TURTLEBOT. Copy this repo to your turtlebot and rosmake this package. It is simply a modified version of turtlebot_navigation to better fit our needs. Changes include, but are not limited to: resetting the default map_file, turning on rgb_processing,
and tinkering with the .yaml configuration files. 

###Running remote_nav
In order for remote_nav to run, several things need to be set up (this may be fixed with a future update)
* turtlebot bringup
* ROS_MASTER_URI must be set to the robot
* Amcl must be running with a valid mapfile. This can be run on Blood by roslaunch custom_nav amcl_demo.launch, which uses betterMap.yaml by default.
* If rviz doesn't see both the map and the video stream, remote_nav won't, either.
* The please run the proper launchfile for the corresponding robot. 
```
roslaunch remote_nav pr2UI.launch
roslaunch remote_nav turtleUI.launch
```
