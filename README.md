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

###Running remote_nav
In order for remote_nav to run, several things need to be set up (this may be fixed with a future update)
* turtlebot bringup
* ROS_MASTER_URI must be set to the turtlebot
* amcl must be running with a valid mapfile
* If rviz doesn't see both the map and the video stream, remote_nav won't, either. 
* If you want to send navigational goals, run the launchfile.  Twist commands do not require the launchfile
