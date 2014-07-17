#!/usr/bin/env python
#vim /etc/puppet/manifests/site.pp

import tf
import rospy
import datetime
import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal
from ar_track_alvar.msg import AlvarMarkers
from geometry_msgs.msg import PointStamped


client = None 
def tf_callback(data):
	print ( "Rospy time: " + str(rospy.get_time()) )
	print ( "datetime: " + str( datetime.datetime.now()) ) 
def marker_callback(data):
	if len(data.markers) > 0:
		# marker = None
		# for (mark in data.markers):
		# 	if mark.id = 1:
		# 		marker = mark
		# if marker == None:
		# 	break
		marker = data.markers[0]
		goal = PointHeadGoal()
		#the point to be looking at is expressed in the "base_link" frame
		point = PointStamped()
		point.header.stamp = rospy.Time.now()
		point.header.frame_id = marker.header.frame_id
		point.point.x = marker.pose.pose.position.x
		point.point.y = marker.pose.pose.position.y 
		point.point.z = marker.pose.pose.position.z
		goal.target = point

		#we want the X axis of the camera frame to be pointing at the target
		goal.pointing_frame = "high_def_frame"
		goal.pointing_axis.x = 1
		goal.pointing_axis.y = 0
		goal.pointing_axis.z = 0
		# goal.min_duration = rospy.Duration(2.0)
		# goal.max_velocity = 1.0
		client.send_goal(goal)
		# client.wait_for_result()




if __name__ == '__main__':
	rospy.init_node('timeTester2')
	client = actionlib.SimpleActionClient("/head_traj_controller/point_head_action", PointHeadAction)
	client.wait_for_server()	
	while not rospy.is_shutdown():
		goal = PointHeadGoal()
		#the point to be looking at is expressed in the "base_link" frame
		point = PointStamped()
		point.header.stamp = rospy.Time.now()
		point.header.frame_id = "/l_gripper_led_frame"
		point.point.x = 0
		point.point.y = 0 
		point.point.z = 0
		goal.target = point

		#we want the X axis of the camera frame to be pointing at the target
		goal.pointing_frame = "high_def_frame"
		goal.pointing_axis.x = 1
		goal.pointing_axis.y = 0
		goal.pointing_axis.z = 0
		# goal.min_duration = rospy.Duration(2.0)
		# goal.max_velocity = 1.0
		client.send_goal(goal)	
		client.wait_for_result()
			
	# marker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, marker_callback)
	rospy.spin()

