#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import cv, cv2
import numpy
import matplotlib
import copy

class ColorViewer():
    def __init__(self, topic):
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.image_callback)
        self.pub = rospy.Publisher(topic + '_viewer', Image)

    def image_callback(self, image_in):
        """ Get image to which we're subscribed. """
        image_cv = self.bridge.imgmsg_to_cv(image_in, 'bgr8')
        image_cv2 = numpy.array(image_cv, dtype=numpy.uint8)
        image_hsv = cv2.cvtColor(image_cv2, cv2.COLOR_BGR2HSV)
        image_hsv = cv2.blur(image_hsv, (3,3))

        h = [150, 480-150]  # max 480
        w = [200, 640-200]  # max 640

        cv2.rectangle(image_cv2, (w[0], h[0]), (w[1], h[1]), 255)
        
        selected = image_hsv[h[0]:h[1], w[0]:w[1], :]
        print selected.shape
        print numpy.amin(numpy.amin(selected, 0), 0)
        print numpy.mean(numpy.mean(selected, 0), 0)
        print numpy.amax(numpy.amax(selected, 0), 0)
        print ''

        # Convert back to ROS Image msg
        image_cv = cv.fromarray(image_cv2)
        image_out = self.bridge.cv_to_imgmsg(image_cv, 'bgr8')
        self.pub.publish(image_out)



if __name__ == '__main__':

    rospy.init_node('color_viewer')

    colorViewer = ColorViewer('/camera/rgb/image_color')

    rospy.spin()
