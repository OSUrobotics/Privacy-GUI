#!/usr/bin/env python

import rospy
import time
import tf
	

# Publishes our "/start" frame, with parent "/map", used to indicate the beginning of the hallway the pr2/turtlebot will navigate.
#MAIN -----------------------------------------------------------------------------------
if __name__ == '__main__':
	rospy.init_node('startFrame', log_level=rospy.DEBUG)
	
	broadcaster = tf.TransformBroadcaster()
	while not rospy.is_shutdown():
## WARNING Do not use absolute directories. Please place this csv file in the package somewhere. It causes errors.
		frameLocations = rospy.get_param('framePublisher/frameLocations', "/nfs/attic/smartw/users/reume02/catkin_ws/src/privacy-interfaces/privacy/config/frames/newThing.csv")
		#frameLocations is the location of a csv file that has each row containing the name and 
		#pose of our objects relative to /map.
		try: 
			frame = "/start"
			parentFrame = "/map"
			#TODO: figure out correct placement and orientation math.
			position = (-0.498853027821, 0.0277761220932, 0.0)
			rotation = (0.0, 0.0, -0.0041319729078, 1.0)
			broadcaster.sendTransform(position,rotation,rospy.Time.now(),frame,parentFrame)	
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, LookupError):
			continue	
	rospy.spin()
