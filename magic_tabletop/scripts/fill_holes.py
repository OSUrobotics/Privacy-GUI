#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv, cv2
import numpy


class HoleFiller():
    def __init__(self, topic_base, topic_mask, topic_out):
        self.bridge = CvBridge()
        self.image_out = Image()
        self.have_mask = False
        self.need_to_publish = False
        rospy.Subscriber(topic_base, Image, self.base_callback)
        rospy.Subscriber(topic_mask, Image, self.mask_callback)
        self.pub = rospy.Publisher(topic_out, Image)

        self.publish()

    def mask_callback(self, mask):
        image_cv = self.bridge.imgmsg_to_cv(mask, 'bgr8')
        image_cv2 = numpy.array(image_cv, dtype=numpy.uint8)
        image_cv2 = image_cv2[:, :, 0]
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (50, 50))
        image_filled = cv2.morphologyEx(image_cv2, cv2.MORPH_CLOSE, kernel, borderValue=255)

        image_filled.astype(bool)
        image_filled = numpy.logical_not(image_filled)
        self.mask = image_filled

        self.have_mask = True


    def base_callback(self, image_in):
        if self.have_mask:
            image_cv = self.bridge.imgmsg_to_cv(image_in, 'bgr8')
            image_cv2 = numpy.array(image_cv, dtype=numpy.uint8)
            
            # Filter
            for dim in range(3): image_cv2[:, :, dim] *= self.mask    # take out filtered pixels
            image_cv2[:, :, 1] += numpy.logical_not(self.mask)*255  # turn 'em green
            
            image_filled_cv = cv.fromarray(image_cv2)
            self.image_out = self.bridge.cv_to_imgmsg(image_filled_cv, 'bgr8')
            self.need_to_publish = True

    def publish(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.need_to_publish:
                self.pub.publish(self.image_out)
                self.need_to_publish = False
                
            r.sleep()



if __name__ == '__main__':

    rospy.init_node('fill_holes')

    filler = HoleFiller('/camera/rgb/image_color',
                        '/camera/rgb/image_mask',
                        '/camera/rgb/image_filtered')
