#!/usr/bin/env python
import roslib
roslib.load_manifest('privacy')
# export ROS_MASTER_URI=http://10.214.152.11:11311
# roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/turtlebot/map2.yaml

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

from privacy.msg import PoseMarkers


#Global variables referring to image manipulation types. Variables only necessary for good organization.
REDACT = "REDACT"	#Black out the image entirely.
BLUR = "BLUR"	  	#apply a guassian blur.
NPR = "NPR"       	#Apply a non-photorealistic filter.
REPLACE = "REPLACE"	#Replace the tagged item with what's around it.
CONTROL = "CONTROL"	#Do nothing to the item.

#herein lies the library that helps me go from 3d to 2d space. Global because I feel like it.
camModel = PinholeCameraModel()


#classes -------------------------------------------------------------------------------

#the marker class stores the ARmarker pattern name, its blur type, and its rotational and translational info.
#also now saves the offset of the rectangle it intends to manipulate.   
class marker: 
	def __init__(self, pattern, manip, position=None, offset=None,offset2=None):
		self.pattern = pattern
		self.manip = manip
		self.position = position
		self.offset = offset
		self.offset2 = offset2

   	#return's the marker's point in the 2d plane.
	def getFirstPoint(self):
		markerPoint = self.position[0]
		offsetPoint = (self.offset[0] + markerPoint[0], self.offset[1] + markerPoint[1], self.offset[2] + markerPoint[2])
		return self.positionToPoint(offsetPoint)

  	#return's the marker's offset2 point in the 2d plane.
	def getSecondPoint(self):
		markerPoint = self.position[0]
		offsetPoint = (self.offset2[0] + markerPoint[0], self.offset2[1] + markerPoint[1], self.offset2[2] + markerPoint[2])
		return self.positionToPoint(offsetPoint)

  	#takes a three-tuple position and returns projected point onto the 2d plane.  
	def positionToPoint(self, position):
		projected = camModel.project3dToPixel(position)
		newX = int(projected[0])
		newY = int(projected[1])
		return (newX, newY)

	def debug(self):
		rospy.logdebug("pattern: {0}, manip: {1}, position: {2}, offset: {3}, offset2: {4}".format(self.pattern, self.manip, self.position, self.offset, self.offset2))

#This class censors objects according to the above manipulation types.
class privacy:
	def __init__(self, config, defaultManip=CONTROL, doRecord=False):
		#subscribe to image info to manipulate and camera_info to get the details needed to perform 
		#3d position to 2d point projections.
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
		self.info_sub  = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.camera_callback)
		self.marker_sub = rospy.Subscriber("pose_markers", PoseMarkers, self.marker_callback)
		self.tf_sub = rospy.Subscriber("/tf", tf.msg.tfMessage, self.tf_callback)

		#we don't want to act on markers if we have none.
		self.hasMarkers = False
		self.weGottaImage = False
	
		#store our record boolean and defaultManipulation style.
		self.doRecord = doRecord
		self.defaultManip = defaultManip
		#bridge to convert the image to cvMAT.
		self.bridge = CvBridge()

		#build the configuration for our markers.
		self.configFile = config
		self.buildConfig(self.configFile)


#callbacks -------------------------------------------------------------------------------     
   	#we only have one callback. It takes in image data, calls ar_pose_marker and then decides
   	#how to manipulate the image data.	
	def image_action(self):
		if self.weGottaImage:
			#Find our markers and act on them.
			if self.hasMarkers:
	  			for myMarker in self.markers:
					manipChoice = myMarker.manip
					if manipChoice == REDACT:
						self.redact(myMarker)
					elif manipChoice == BLUR:
						self.blur(myMarker)		
					elif manipChoice == NPR:
						self.NPR(myMarker)
					elif manipChoice == REPLACE:
						self.replace(myMarker)	
  			#self.drawOrigin()
			#if we want to record, record.
			if self.doRecord:
				self.record(time.time())

			#SHOW ZE IMAGE!
			cv2.imshow("canvas", self.image)
			cv.WaitKey(3)

	#Just update camModel to use the most recent cameraInfo.
	def camera_callback(self, cameraInfo):
		#an object containing the camera's parameters.
		self.cameraInfo = cameraInfo
		#we update our camModel with the cameraInfo to work some magic.
		camModel.fromCameraInfo(cameraInfo)

	#create an array, self.markers, that keeps all markers that were spotted.
	def marker_callback(self, markers):
		#for logic control outside of this callback.
		self.hasMarkers = True
		#an array that stores any caught markers.
		self.markers = []

		hasChair = False
		hasDoor = False
		self.buildConfig(self.configFile)

		for myMarker in markers.markers:
			#a safegaurd against the offsets csv and frames csv not being bijective.
			if not myMarker.header.frame_id in self.config:
				continue
			#the marker's properties are in config already; if it's found, steal that data.
			foundMarker = copy(self.config[myMarker.header.frame_id])
	
			#get marker's position and rotation.
			trans = (myMarker.pose.position.x, myMarker.pose.position.y, myMarker.pose.position.z)
	   		rot = (myMarker.pose.orientation.x, myMarker.pose.orientation.y, myMarker.pose.orientation.z, myMarker.pose.orientation.w)
			position = (trans, rot)
			foundMarker.position = position

			#only keep one door tag or one chair tag.
			if not ("Chair" in foundMarker.pattern and hasChair) and not ("Door" in foundMarker.pattern and hasDoor):
				self.markers.append(foundMarker)
	   		#If we caught a chair/door, then don't bother catching another.
			if "Chair" in foundMarker.pattern:
				hasChair = True
			if "Door" in foundMarker.pattern:
				hasDoor = True
	def image_callback(self, image):
		#convert from a ROS Image message to a cvMAT image to a numpy array.
		middleman = self.bridge.imgmsg_to_cv(image, "bgr8")
		self.image = numpy.asarray(middleman)
		self.weGottaImage = True
	def tf_callback(self, tfmessage):
		for transf in tfmessage.transforms:
			if transf.child_frame_id == "/camera_rgb_optical_frame":
				self.image_action()
		
	# helper / Image Manipulation functions -------------------------------------------------------
	#Calls all of our patterns in settings[] to get their info and build marker objects.
	#not technically a manipulation function, but very useful for good code practice.
	"""def updateMarkers(self, image):
	self.markers = []

	for pattern, manipChoice in settings.iteritems():
		try:
		#Grab a marker's position, if it can be found.
		position = self.listener.lookupTransform(self.cameraInfo.header.frame_id, pattern, rospy.Time(self.cameraInfo.header.stamp.secs))
		rospy.logdebug("{0}: {1}".format(pattern, position))
		#if we find a position, make a marker and put it in our self.markers array.
		myMarker = marker(pattern, manipChoice, position, offset[pattern])
		self.markers.append(myMarker)
		except (tf.LookupException, tf.ConnectivityException):
			rospy.logdebug("whatevs")
"""
	#applies a redaction filter to the object.
	def redact(self, marker):
		markerPoint = marker.getFirstPoint()
		offsetPoint = marker.getSecondPoint()

		cv2.putText(self.image, str(marker.position[0]), marker.positionToPoint(marker.position[0]),cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 255, 0))
		origin = self.positionToPoint((0, 0, marker.position[0][2]))
		cv2.rectangle(self.image, markerPoint, offsetPoint, -1, -1)
	
		#applies a blur filter to the object.
	def blur(self, marker):
		firstPoint = self.refinePoint(marker.getFirstPoint())
		secondPoint = self.refinePoint(marker.getSecondPoint())
		x1 = firstPoint[0]
		y1 = firstPoint[1]
		x2 = secondPoint[0]
		y2 = secondPoint[1]
		try:
			crop_image = self.image[y1:y2, x1:x2]
			crop_image = cv2.GaussianBlur(crop_image,(7,7), 100, 100)
			self.image[y1:y2, x1:x2] = crop_image[:,:]
			cv2.rectangle(self.image, firstPoint, secondPoint, -1)
		except (TypeError):
			pass
		
	#applies a non-photorealistic filter to the object.
	def NPR(self, marker):
		pass
		
	#Attempts to replace the object with what surrounds it.
	def replace(self, marker):
		pass
	#draw origin of the 3D SPACE
	def drawOrigin(self):
		origin = self.positionToPoint((0, 0, 0.1))
		cv2.circle(self.image, origin, 5, (0, 255, 0), -1)

	#takes a 3d position and returns a 3d point. Technically redundant.
	def positionToPoint(self, position):
		projected = camModel.project3dToPixel(position)
		flatPoint = (int(projected[0]), int(projected[1]))
		return flatPoint
	#makes sure a point is not off the screen.
	def refinePoint(self, point):
		newX = point[0]
		newY = point[1]
		if point[1] >= self.image.shape[0] - 1:
			newY = self.image.shape[0] - 1
			#rospy.logwarn("point[0] is >= self.image.shape[1] - 1, so point[0] = {0} and newY = {1}".format(point[0], newY))
		if point[0] >= self.image.shape[1] - 1:
			newX = self.image.shape[1] - 1
		if point[1] < 0:
			newY = 0
		if point[0] < 0:
			newX = 0
		#rospy.logwarn("(newX, newY) = {0}".format( (newX, newY) ))
		return (newX, newY)

	#takes in marker settings from a .csv and stores their properties in config[].
	#config must be in the same order as the pattern file in your launch.
   	def buildConfig(self, fileName):
		self.config = {}

		with open(fileName, 'rb') as csvfile:
			reader = csv.reader(csvfile, delimiter=',', quotechar="'")
			for row in reader:
				pattern = row[0]
				if row[1] == "":
					row[1] = self.defaultManip
				manip = row[1]
				offset = (float(row[2]), float(row[3]), float(row[4]))
				offset2 = (float(row[5]), float(row[6]), float(row[7]))
				configMarker = marker(pattern, manip, None, offset,offset2)
				self.config[pattern] = configMarker

#Video recording function(s) ------------------------------------------------------------
	#records the video stream.
	def record(self,fileName):
		cv2.imwrite("/home/ahubers/catkin_ws/src/privacy/videoJpegs/" + str(fileName) + ".jpeg", self.image)
	
#MAIN -----------------------------------------------------------------------------------
if __name__ == '__main__':
	rospy.init_node('privacy', log_level=rospy.DEBUG)
   	defaultManip = rospy.get_param('privacy/defaultManip', "REDACT")
   	doRecord = rospy.get_param('privacy/doRecord', False)
	config = rospy.get_param('privacy/config', "/home/ahubers/catkin_ws/src/privacy/config/offsets/test1.csv")
	myPrivacy = privacy(config, defaultManip, doRecord)
	
  
	rospy.spin()
