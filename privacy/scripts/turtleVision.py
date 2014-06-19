#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv, cv2
import numpy

from rosCV import rosCV as rcv

class turtleViz():
    def __init__(self, topic):
        # self.defaultManip = rospy.get_param('boundingBox/defaultManip')
        rospy.Subscriber(topic, Image, self.image_callback)
        self.rcv = rcv()
        self.pub = rospy.Publisher('turtleVision', Image)

    def image_callback(self, image_in):
        """ Get image to which we're subscribed. """

        # Import and convert
        image_cv2 = self.rcv.toCv2(image_in)
        topLeft = (1,1)
        bottomRight = (200,200)
        image_cv2 = self.rcv.redact(image_cv2, (1,1), (200,200))
        self.rcv.imshow(image_cv2)

        # Convert back to ROS Image msg
        image_out = self.rcv.toRos(image_cv2)
        self.pub.publish(image_out)

if __name__ == '__main__':

    rospy.init_node('turtleVision')

    boundingBox = turtleViz('/camera/rgb/image_color_repub')

    rospy.spin()
