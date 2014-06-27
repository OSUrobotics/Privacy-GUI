#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import cv, cv2
import numpy
import matplotlib
import copy

# import code # FOR TESTING

class Bound():
	def __init__(self, topic):
		self.bridge = CvBridge()
		rospy.Subscriber(topic, Image, self.image_callback)
		self.pub = rospy.Publisher(topic + '_filtered', Image)

	def image_callback(self, image_in):
		# Import and convert
		image_cv = self.bridge.imgmsg_to_cv(image_in, 'bgr8')
		image_cv2 = numpy.array(image_cv, dtype=numpy.uint8)
		image_hsv = cv2.cvtColor(image_cv2, cv2.COLOR_BGR2HSV)
		image_hsv = cv2.blur(image_hsv, (5, 5))

		# Make binary image of pinkness
		lowerb = numpy.array((50, 100,100))
		upperb = numpy.array((80,255, 255))
		# lowerb = numpy.array((130, 50, 0))
		# upperb = numpy.array((175, 255, 255))
		# is_pink = cv2.inRange(image_hsv, numpy.array((130, 50, 0)), numpy.array((175, 255, 255)))


		# Make binary image of pinkness
		# is_pink = cv2.inRange(image_hsv, numpy.array((50, 92, 50)), numpy.array((132, 231, 187)))
		is_green = cv2.inRange(image_hsv, lowerb, upperb)

		green = copy.deepcopy(image_cv2)
		for dim in range(3): green[:, :, dim] *= is_green / 255
		green_avg = numpy.sum(numpy.sum(green, 0), 0) / numpy.sum(numpy.sum(is_green / 255, 0), 0)
		green_avg = tuple([int(green_avg[0]), int(green_avg[1]), int(green_avg[2])])
		# print green_avg
		#code.interact(local=locals())
		# Manipulate binary image
		contours, hierarchy = cv2.findContours(is_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		#code.interact(local=locals())
		#print contours

		max_area = 0
		for index, contour in enumerate(contours):
			area = cv2.contourArea(contour)
			if area > max_area:
				max_area = area
				best_index = index

		try:
			best_contour = contours[best_index]
			rospy.loginfo('Best contour was contour #{0} with area of {1}'.format(best_index, max_area))
			cv2.drawContours(image_cv2, contours, best_index, green_avg, thickness=-1)  # fill in the largest pink blob
		except(UnboundLocalError):
			pass
		#cv2.drawContours(image_cv2, contours, best_index, (0,0,0))  # draw black line around largest pink blob

		
		# Apply binary image to full image
		# image_hsv[:,:] = green_avg
		for dim in range(3): image_hsv[:,:,dim] *= is_green / 255

		# Convert back to ROS Image msg
		# image_hsv_float = (pink.astype(float) + 1) / 256
		# image_rgb = ((matplotlib.colors.hsv_to_rgb(image_hsv_float) * 256) - 1).astype('uint8')
		# image_cv2 = cv2.cvtColor(image_hsv, cv2.COLOR_HSV2BGR)


		image_cv = cv.fromarray(image_cv2)
		image_out = self.bridge.cv_to_imgmsg(image_cv, 'bgr8')
		self.pub.publish(image_out)




if __name__ == '__main__':

	rospy.init_node('boundingBox')

	boundingBox = Bound('/camera/rgb/image_color')

	rospy.spin()
