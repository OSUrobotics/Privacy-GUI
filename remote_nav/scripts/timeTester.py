#!/usr/bin/env python
#vim /etc/puppet/manifests/site.pp

import tf
import rospy
import datetime

def tf_callback(data):
	print ( "Rospy time: " + str(rospy.get_time()) )
	print ( "datetime: " + str( datetime.datetime.now()) ) 

if __name__ == '__main__':
	rospy.init_node('timeTester2')
	tf_sub = rospy.Subscriber("/tf", tf.msg.tfMessage, tf_callback)
	rospy.spin()

