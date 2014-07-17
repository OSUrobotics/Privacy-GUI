Privacy Interfaces - GROOVY BRANCH
==================

Welcome to the Privacy Interfaces repo!  Please check out the Wiki for current information and project goals. 

Visual interfaces and filters for enforcing privacy in the home.

Authors: 
* Matt (Physical Markers)
* Suzanne (Interfaces)
* Alex (Localization)
* Penn (Filters and Image Products)

##Why Groovy?
* The current version of the rviz library used in python does not support hydro
* Mixing groovy and hydro causes crashes and segfaults
* If you are running any of these packages, **have a separate groovy workspace**

##Package list and info
* gui  - The first iteration of the attempt at the gui.  Not currently used but has some nice buttons.
* remote_nav - The real package and current implementation of our UI
* map_registration - not a ROS package but a pyQT program which is used for map registration

##custom_nav package
The custom_nav package is to be used ON THE TURTLEBOT. Copy this repo to your turtlebot and rosmake this package. It is simply a modified version of turtlebot_navigation to better fit our needs. Changes include, but are not limited to: resetting the default map_file, turning on rgb_processing,
and tinkering with the .yaml configuration files. 

###Running remote_nav
In order for remote_nav to run, several things need to be set up (this may be fixed with a future update)
* turtlebot bringup
* ROS_MASTER_URI must be set to the turtlebot
* Amcl must be running with a valid mapfile. This can be run on Blood by roslaunch custom_nav amcl_demo.launch, which uses betterMap.yaml by default.
* If rviz doesn't see both the map and the video stream, remote_nav won't, either. 
* If you want to send navigational goals, run the launchfile.  Twist commands do not require the launchfile
