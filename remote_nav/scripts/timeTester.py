#!/usr/bin/env python
import tf
import rospy
import datetime

def tf_callback(data):
	print ( "Rospy time: " + rospy.Time.now())
	print ( "datetime: " + datetime.datetime.now() ) 

if __name__ == '__main__':
	rospy.init_node('timeTester.py')
	tf_sub = rospy.Subscriber("/tf", tf.msg.tfMessage, tf_callback)
	rospy.spin()

