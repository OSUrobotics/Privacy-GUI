#!/usr/bin/env python
#vim /etc/puppet/manifests/site.pp

import tf
import rospy
from sensor_msgs.msg import Range

if __name__ == '__main__':
	rospy.init_node('rangePub')
	pub = rospy.Publisher('/vision_range', Range)
	r = rospy.Rate(50)
	print "gonna start publishin'"
	while not rospy.is_shutdown():
		vision_range = Range()
		vision_range.header.frame_id = "/high_def_frame"
		# vision_range.header.stamp = rospy.Time.now()
		vision_range.radiation_type = 0
		vision_range.field_of_view = 0.5
		vision_range.min_range = 0.0
		vision_range.max_range = 100.0
		vision_range.range = 1.0
		pub.publish(vision_range)


	# tf_sub = rospy.Subscriber("/tf", tf.msg.tfMessage, tf_callback)
	rospy.spin()
