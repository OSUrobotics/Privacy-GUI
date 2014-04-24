#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import cv, cv2
import numpy
import matplotlib
import copy

import code # FOR TESTING

class MatFilter():
    def __init__(self, topic):
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.image_callback)
        self.pub = rospy.Publisher(topic + '_filtered', Image)

    def image_callback(self, image_in):
        """ Get image to which we're subscribed. """

        # Import and convert
        image_cv = self.bridge.imgmsg_to_cv(image_in, 'bgr8')
        image_cv2 = numpy.array(image_cv, dtype=numpy.uint8)
        image_hsv = cv2.cvtColor(image_cv2, cv2.COLOR_BGR2HSV)
        image_hsv = cv2.blur(image_hsv, (5, 5))

        # Make binary image of pinkness
        #is_pink = cv2.inRange(image_hsv, numpy.array((130, 50, 120)), numpy.array((175, 85, 200)))
        is_pink = cv2.inRange(image_hsv, numpy.array((130, 50, 0)), numpy.array((175, 255, 255)))
        pink = copy.deepcopy(image_cv2)
        for dim in range(3): pink[:, :, dim] *= is_pink / 255
        pink_avg = numpy.sum(numpy.sum(pink, 0), 0) / numpy.sum(numpy.sum(is_pink / 255, 0), 0)
        pink_avg = tuple([int(pink_avg[0]), int(pink_avg[1]), int(pink_avg[2])])
        print pink_avg
        #code.interact(local=locals())

        # Manipulate binary image
        contours, hierarchy = cv2.findContours(is_pink, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        #code.interact(local=locals())
        #print contours


        max_area = 0
        for index, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                best_index = index

        best_contour = contours[best_index]
        rospy.loginfo('Best contour was contour #{0} with area of {1}'.format(best_index, max_area))
        cv2.drawContours(image_cv2, contours, best_index, pink_avg, thickness=-1)  # fill in the largest pink blob
        #cv2.drawContours(image_cv2, contours, best_index, (0,0,0))  # draw black line around largest pink blob

        
        # Apply binary image to full image
        for dim in range(3): image_hsv[:,:,dim] *= is_pink / 255

        # Convert back to ROS Image msg
        #image_hsv_float = (pink.astype(float) + 1) / 256
        #image_rgb = ((matplotlib.colors.hsv_to_rgb(image_hsv_float) * 256) - 1).astype('uint8')
        #image_cv2 = cv2.cvtColor(image_hsv, cv2.COLOR_HSV2BGR)
        image_cv = cv.fromarray(image_cv2)
        image_out = self.bridge.cv_to_imgmsg(image_cv, 'bgr8')
        self.pub.publish(image_out)



if __name__ == '__main__':

    rospy.init_node('magic_mat')

    matFilter = MatFilter('/camera/rgb/image_color')

    rospy.spin()
