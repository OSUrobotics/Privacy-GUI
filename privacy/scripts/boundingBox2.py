#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import cv, cv2
import numpy
import matplotlib
import copy

from rosCV import rosCV as rcv

# import code # FOR TESTING

class Bound():
    def __init__(self, topic):
        # self.defaultManip = rospy.get_param('boundingBox/defaultManip')
        rospy.Subscriber(topic, Image, self.image_callback)
        self.rcv = rcv()
        self.pub = rospy.Publisher(topic + '_filtered', Image)

    def image_callback(self, image_in):
        """ Get image to which we're subscribed. """

        # Import and convert
        image_cv2 = self.rcv.toCv2(image_in)
        image_hsv = cv2.cvtColor(image_cv2, cv2.COLOR_BGR2HSV)
        image_hsv = cv2.blur(image_hsv, (5, 5))

        # Make binary image of pinkness
        is_pink = cv2.inRange(image_hsv, numpy.array((130, 50, 0)), numpy.array((175, 255, 255)))
        pink = copy.deepcopy(image_cv2)

        # Manipulate binary image
        contours, hierarchy = cv2.findContours(is_pink, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        #Just finds the best contour.
        max_area = 0
        for index, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                best_index = index

        #creates a bounding box and finds the points of this best contour.
        best_contour = contours[best_index]
        maxCnt = best_contour.max(axis=0)
        minCnt = best_contour.min(axis=0)

        maxCntTuple = tuple(maxCnt[0])
        minCntTuple = tuple(minCnt[0])

        topLeft = (minCntTuple[0], minCntTuple[1])
        bottomRight = (maxCntTuple[0], maxCntTuple[1])

        image_cv2 = self.rcv.npr(image_cv2,topLeft,bottomRight)
        self.rcv.imshow(image_cv2)

        # Convert back to ROS Image msg
        image_out = self.rcv.toRos(image_cv2)
        self.pub.publish(image_out)



if __name__ == '__main__':

    rospy.init_node('boundingBox')

    boundingBox = Bound('/camera/rgb/image_color')

    rospy.spin()
