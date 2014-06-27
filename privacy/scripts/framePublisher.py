#!/usr/bin/env python
# import roslib
# roslib.load_manifest('privacy')
# export ROS_MASTER_URI=http://10.214.152.11:11311

import rospy
import sys
import csv 
import time
import tf
	

#This node only publishes frames, which are picked up by frameControl for further processing.
#it basically publishes frames of our desired objects relative to /map so that they can 
#MAIN -----------------------------------------------------------------------------------
if __name__ == '__main__':
	rospy.init_node('framePublisher', log_level=rospy.DEBUG)
	
	broadcaster = tf.TransformBroadcaster()
	while not rospy.is_shutdown():
		frameLocations = rospy.get_param('framePublisher/frameLocations', "/nfs/attic/smartw/users/reume02/catkin_ws/src/privacy-interfaces/privacy/config/frames/newThing.csv")
		#frameLocations is the location of a csv file that has each row containing the name and 
		#pose of our objects relative to /map.
		with open(frameLocations, 'rb') as csvfile:
			reader = csv.reader(csvfile, delimiter=',', quotechar="'")
			for row in reader:
				try: 
					frame = row[0]
					parentFrame = row[1]
					position = (float(row[2]), float(row[3]), float(row[4]))
					rotation = (float(row[5]), float(row[6]), float(row[7]), float(row[8]))
					#broadcaster.sendTransform(position,rotation,rospy.Time.now(),frame,"/odom")
					broadcaster.sendTransform(position,rotation,rospy.Time.now(),frame,parentFrame)	
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, LookupError):
					continue	
	rospy.spin()
