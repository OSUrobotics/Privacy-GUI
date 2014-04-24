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
        image_cv = self.bridge.imgmsg_to_cv(image_in, 'bgr8')
        image_cv2 = numpy.array(image_cv, dtype=numpy.uint8)
        #image_float = (image_cv2.astype(float) + 1) / 256
        #image_hsv = ((matplotlib.colors.rgb_to_hsv(image_float) * 256) - 1).astype('uint8')
        image_hsv = cv2.cvtColor(image_cv2, cv2.COLOR_BGR2HSV)
        image_hsv = cv2.blur(image_hsv, (5, 5))

        #is_pink = cv2.inRange(image_hsv, numpy.array((130, 50, 120)), numpy.array((175, 85, 200)))
        is_pink = cv2.inRange(image_hsv, numpy.array((130, 50, 120)), numpy.array((175, 255, 255)))
        
        #code.interact(local=locals())

        for dim in range(3): image_hsv[:,:,dim] *= is_pink / 255
        #print is_pink

        #is_pink = (image_hsv[:, :, 0] > 200) & (image_hsv[:, :, 0] < 240) & (image_hsv[:, :, 1] > 35) & (image_hsv[:, :, 2] > 100)
        #pink = copy.deepcopy(image_float)
        #for dim in range(3): pink[:, :, dim] = is_pink*1.0
        #pink = cv2.medianBlur(pink, 5)
        """
        contours = cv2.findContours(is_pink, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        print contours
        
        # finding contour with maximum area and store it as best_cnt
        max_area = 10000
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                best_contour = contour

        best_index = contours.index(best_contour)
        rospy.loginfo('Best contour was contour #{0} with area of {1}'.format(best_index, max_area))
        cv2.drawContours(image_cv2, contours, best_index, numpy.array([0,0,0]))  # draw black line around largest pink blob
        """
        
        #kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        #pink = cv2.morphologyEx(pink, cv2.MORPH_CLOSE, kernel, borderValue=255, iterations=1)

        

        #pink = copy.deepcopy(image_hsv)
        #for dim in range(3): pink[:, :, dim] *= is_pink
        #pink_avg = numpy.mean(numpy.mean(pink, 0), 0)  # mean HSV vector for pink bits of the image
        #print pink_avg

        #for dim in range(3): image_cv2[:, :, dim] = numpy.uint8(green)*255
        
        # Convert back to ROS Image msg
        #image_hsv_float = (pink.astype(float) + 1) / 256
        #image_rgb = ((matplotlib.colors.hsv_to_rgb(image_hsv_float) * 256) - 1).astype('uint8')
        image_cv2 = cv2.cvtColor(image_hsv, cv2.COLOR_HSV2BGR)
        image_cv = cv.fromarray(image_cv2)
        image_out = self.bridge.cv_to_imgmsg(image_cv, 'bgr8')
        self.pub.publish(image_out)



if __name__ == '__main__':

    rospy.init_node('magic_mat')

    matFilter = MatFilter('/camera/rgb/image_color')

    rospy.spin()
