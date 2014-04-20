#!/usr/bin/env python

import roslib; roslib.load_manifest('tunnel_vision')

import rospy
import cv
import cv2
import numpy
import sys
import csv 
import time
import inspect
from copy import copy

#for message filtering
import tf
import message_filters

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel

#herein lies the library that helps me go from 3d to 2d space. Global because I feel like it.
camModel = PinholeCameraModel()


class TunnelVision:
	def __init__(self, image_topic="/camera/rgb/image_raw"):

		#subscribe to image info to manipulate and camera_info to get the details needed to perform 
		#3d position to 2d point projections.
		self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
	
		#bridge to convert the image to cvMAT.
		self.bridge = CvBridge()


	def image_callback(self, image):
		#convert from a ROS Image message to a cvMAT image to a numpy array.
		middleman = self.bridge.imgmsg_to_cv(image, "bgr8")
		self.image = numpy.asarray(middleman)
		
		self.tunnel_blur()		

		#SHOW ZE IMAGE!
		cv2.imshow("canvas", self.image)
		cv2.waitKey(3)

	def tunnel_blur(self):
		[height, width, depth] = self.image.shape
		dh = round(height / 8)
		dw = round(width / 8)

		# Seperate out successively narrower fields of view
		image_100 = copy(self.image)
		image_75 = copy(self.image[dh:height-dh, dw:width-dw])
		image_50 = copy(self.image[2*dh:height-2*dh, 2*dw:width-2*dw])
		image_25 = copy(self.image[3*dh:height-3*dh, 3*dw:width-3*dw])

		# Apply filters
		image_100 = cv2.GaussianBlur(image_100, (21, 21), 100, 100)
		image_75 =  cv2.GaussianBlur(image_75,  (11, 11), 100, 100)
		image_50 =  cv2.GaussianBlur(image_50,  ( 5,  5), 100, 100)

		# Re-compile image
		self.image = image_100
		self.image[dh:height-dh, dw:width-dw] = image_75
		self.image[2*dh:height-2*dh, 2*dw:width-2*dw] = image_50
		self.image[3*dh:height-3*dh, 3*dw:width-3*dw] = image_25
		
	
if __name__ == '__main__':

	rospy.init_node('tunnel_vision', log_level=rospy.DEBUG)

	# image_topic = rospy.get_param('privacy/image_topic');
	image_topic = '/camera/rgb/image_raw'
	myTunnelVision = TunnelVision(image_topic)

	rospy.spin()
