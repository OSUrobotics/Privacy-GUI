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

        lowerb = numpy.array((130, 50, 0))
        upperb = numpy.array((175, 255, 255))
        contours = self.rcv.colorContours(image_cv2, lowerb, upperb)

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
        faces = self.rcv.findFaces(image_cv2)
        for x1, y1, x2, y2 in faces:
            topLeft = (x1, y1)
            bottomRight = (x2, y2)
            image_cv2 = self.rcv.redact(image_cv2,topLeft,bottomRight)
        self.rcv.imshow(image_cv2)

        # Convert back to ROS Image msg
        image_out = self.rcv.toRos(image_cv2)
        self.pub.publish(image_out)



if __name__ == '__main__':

    rospy.init_node('boundingBox')

    boundingBox = Bound('/camera/rgb/image_color')

    rospy.spin()
