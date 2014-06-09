#!/usr/bin/env python
import roslib
roslib.load_manifest('privacy')
# export ROS_MASTER_URI=http://10.214.152.11:11311

import rospy
import numpy
import sys
import csv 
import time
import tf
from copy import copy
#just an array of of poseStamped's
from privacy.msg import PoseMarkers
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge

#rosparam set /framePublisher/frameLocations /home/ahubers/catkin_ws/src/privacy/config/frames/slowAngular2.csv
#rosparam set /framePublisher/frameControl /home/ahubers/catkin_ws/src/privacy/config/frames/slowAngular2.csv



class frameControl:

	def __init__(self, depthThreshold, image_topic):
		self.listener = tf.TransformListener()
		self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
		self.info_sub  = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.camera_callback)
		self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
		self.depth_camera_info = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.depthcamera_callback)
		
		self.marker_pub = rospy.Publisher('pose_markers', PoseMarkers)
		self.camModel = PinholeCameraModel()
		#bridge to convert the image to cvMAT.
		self.bridge = CvBridge()
		
		#Do we want to try for occlusion? This isn't working properly so it's disabled via the script and not launch file for the time being.
		self.doOcclusion = False
		#the maximum distance between a point's depth in the camera and a marker's depth we wish to allow.
		self.threshold = depthThreshold
		
#CALLBACKS -------------------------------------------------------------------
	def image_callback(self, image):
		#an array of markers that we'll publish.
		self.markers = PoseMarkers()
		#grab frameLocations every time image_callback is called so that 'frameControls/frameLocations' can be changed dynamically.
		frameLocations = rospy.get_param('frameControl/frameLocations', "/home/ahubers/catkin_ws/src/privacy/config/frames/test.csv")
		#perform the transforms necessary to get our positions of frames.
		with open(frameLocations, 'rb') as csvfile:
			reader = csv.reader(csvfile, delimiter=',', quotechar="'")
			for row in reader:
				newMarker = PoseStamped()
				newMarker.header.frame_id = row[0]
				try:
					(trans, rot) = self.listener.lookupTransform(self.cameraInfo.header.frame_id, newMarker.header.frame_id, rospy.Time(0))
					newMarker.pose.position.x = trans[0]
					newMarker.pose.position.y = trans[1]
					newMarker.pose.position.z = trans[2]
				
					newMarker.pose.orientation.x = rot[0]
					newMarker.pose.orientation.y = rot[1]
					newMarker.pose.orientation.z = rot[2]
					newMarker.pose.orientation.w = rot[3]
					
					#Disabling because it's causing errors.
					# #projected point from the 3d position to the image frame.
					# projected = self.camModel.project3dToPixel(trans)
					
					# #correct it if the point's off the frame, also convert the pixels from float to int.
					# projected = self.refinePoint(projected)
					# #z-distance from the camera to the marker.
					# markerDepth = trans[2]
					# #z-distance from the camera to the point on the screen corresonding to the marker.
					# pointDepth = self.depth_image[projected[0]][projected[1]]/1000
					# pointDepth = self.averagePoint(projected)/1000
					
					#if the frame is behind us, or there's occlusion, ignore it.
					if newMarker.pose.position.z >= 0:
						if not self.doOcclusion or not self.occlusionCheck(markerDepth, pointDepth):
							self.markers.markers.append(newMarker)
							
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, AttributeError):
					rospy.logdebug("transform failed for {0}".format(newMarker.header.frame_id))
			self.marker_pub.publish(self.markers)	
				
	#we want the frame_id of the camera.
	def camera_callback(self, cameraInfo):
		#an object containing the camera's parameters.
		self.cameraInfo = cameraInfo
		
	def depthcamera_callback(self, depthCamInfo):
		self.depthCamInfo = depthCamInfo
		#we update our depthcamModel with the cameraInfo to work some magic.
		self.camModel.fromCameraInfo(self.depthCamInfo)
		
	#get the depth image so we can check for occlusion.
	def depth_callback(self, image):
		#convert from a ROS Image message to a cvMAT image to a numpy array.
		middleman = self.bridge.imgmsg_to_cv(image, "16UC1")
		self.depth_image = numpy.asarray(middleman)
		
#HELPER METHODS --------------------------------------------------------------
	#check if there's something blocking the frame. True if there is, false if isn't.
	def occlusionCheck(self, markerDepth, pointDepth):
		rospy.logwarn("markerDepth - pointDepth = {0} - {1} = {2}".format(markerDepth, pointDepth, (markerDepth - pointDepth)))
		if abs(markerDepth - pointDepth) > self.threshold:
			return True
		return False
		
		#makes sure a point is not off the screen.
	def refinePoint(self, point):
		newX = int(point[0])
		newY = int(point[1])
		shape = self.depth_image.shape
		if point[1] >= shape[0] - 1:
			newY = shape[0] - 1
			#rospy.logwarn("point[0] is >= self.image.shape[1] - 1, so point[0] = {0} and newY = {1}".format(point[0], newY))
		if point[0] >= shape[1] - 1:
			newX = shape[1] - 1
		if point[1] < 0:
			newY = 0
		if point[0] < 0:
			newX = 0
		#rospy.logwarn("(newX, newY) = {0}".format( (newX, newY) ))
		return (newY, newX)
		
	#returns an average of the points around a pixel.
	def averagePoint(self, pixel):
		x = int(pixel[0])
		y = int(pixel[1])
		
		try:
			averagingArray = self.depth_image[x-1:x+1, y-1:y+1]
			for (x,y), value in numpy.ndenumerate(averagingArray):
				averagingArray[x,y] = float(value)
			return float(numpy.mean(averagingArray))
		except(IndexError):
			pass
		return float(self.depth_image[x,y])
		
	
		
#MAIN ------------------------------------------------------------------------
if __name__ == '__main__':
	rospy.init_node('frameControl', log_level=rospy.DEBUG)
	frameLocations = rospy.get_param('frameControl/frameLocations', "/home/ahubers/catkin_ws/src/privacy/config/frames/test.csv")
	threshold = rospy.get_param('frameControl/depthThreshold', "1.0")
	image_topic = rospy.get_param('privacy/image_topic');
	myFrameControl = frameControl(threshold, image_topic)		
	rospy.spin()
