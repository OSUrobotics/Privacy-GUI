#!/usr/bin/env python
#
# Makes stuff higher than the floor disappear. Steps:
# 1) Grabs a Kinect PointCloud2 msg
# 2) Finds the dominant plane -- the floor, if the Kinect is pointed down
# 3) Projects all points above that (floor) plane into the floor
# 4) Paints those points a different color
# 5) Converts back to an Image msg, which is displayed to the user as video

import rospy
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import cv, cv2
import numpy
import matplotlib
import copy

from magic_tabletop.srv import *


class MatFilter():
    def __init__(self, topic):
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.image_callback)
        self.pub = rospy.Publisher(topic + '_filtered', Image)

    def image_callback(self, image_in):
        """ Get image to which we're subscribed. """
        image_cv = self.bridge.imgmsg_to_cv(image_in, 'bgr8')
        image_cv2 = numpy.array(image_cv, dtype=numpy.uint8)
        image_float = (image_cv2.astype(float) + 1) / 256
        image_hsv = ((matplotlib.colors.rgb_to_hsv(image_float) * 256) - 1).astype('uint8')
        
        #print image_cv2.shape
        #print image_hsv[250, 250, :]  # print something in the middle


        is_pink = (image_hsv[:, :, 0] > 200) & (image_hsv[:, :, 0] < 240) & (image_hsv[:, :, 1] > 30)
        pink = copy.deepcopy(image_hsv)
        for dim in range(3): pink[:, :, dim] *= is_pink
        pink_avg = numpy.mean(numpy.mean(pink, 0), 0)  # mean HSV vector for pink bits of the image
        print pink_avg

        #for dim in range(3): image_cv2[:, :, dim] = numpy.uint8(green)*255
        
        # Convert back to ROS Image msg
        image_hsv_float = (pink.astype(float) + 1) / 256
        image_rgb = ((matplotlib.colors.hsv_to_rgb(image_hsv_float) * 256) - 1).astype('uint8')
        image_cv = cv.fromarray(image_rgb)
        image_out = self.bridge.cv_to_imgmsg(image_cv, 'bgr8')
        self.pub.publish(image_out)



if __name__ == '__main__':

    rospy.init_node('magic_mat')

    matFilter = MatFilter('/camera/rgb/image_color')

    rospy.spin()
