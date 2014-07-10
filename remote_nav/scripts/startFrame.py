#!/usr/bin/env python

import rospy
import time
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

position = None
rotation = None

def initialpose_callback(initialpose):
	global position, rotation
	position = (initialpose.pose.pose.position.x,initialpose.pose.pose.position.y,initialpose.pose.pose.position.z)
	rotation = (initialpose.pose.pose.orientation.x,initialpose.pose.pose.orientation.y,initialpose.pose.pose.orientation.z,initialpose.pose.pose.orientation.w)
# Publishes our "/start" frame, with parent "/map", used to indicate the beginning of the hallway the pr2/turtlebot will navigate.
#MAIN -----------------------------------------------------------------------------------
if __name__ == '__main__':
	rospy.init_node('startFrame', log_level=rospy.DEBUG)
	rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, initialpose_callback)
	
	broadcaster = tf.TransformBroadcaster()
	while not rospy.is_shutdown():
		r = rospy.Rate(100.0)
		try: 
			if (position and rotation):
				frame = "/start"
				parentFrame = "/map"
				broadcaster.sendTransform(position,rotation,rospy.Time.now(),frame,parentFrame)
				r.sleep()	
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, LookupError):
			continue	
	rospy.spin()
"""
Requested time 1,405,019,497.021159172
 but earliest  1,405,019,543.295389891
"""