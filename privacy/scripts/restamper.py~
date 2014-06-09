#!/usr/bin/env python

#I would first like to announce that this is the ugliest piece of code in history
#and I regret very much the thought that anyone else must read this or have to 
#use the underlying structure pushing this code. In short, I am intercepting every
#single topic published by my bag file, changing its header to NOW, and then republishing it.
#Lord have mercy on my soul.

import roslib
roslib.load_manifest('privacy')
import rospy
import tf
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry

rospy.init_node("restamper")
tf_pub= rospy.Publisher("tf",tf.msg.tfMessage)
depth_pub = rospy.Publisher("camera/depth/image_raw",Image)
depth_camera_pub = rospy.Publisher("camera/depth/camera_info", CameraInfo)
rgb_pub = rospy.Publisher("camera/rgb/image_raw",Image)
camera_pub = rospy.Publisher("camera/rgb/camera_info",CameraInfo)
map_pub = rospy.Publisher("/map", OccupancyGrid)
odom_pub = rospy.Publisher("/odom",Odometry)
scan_pub = rospy.Publisher("/scan", LaserScan)

def tf_callback(tfmessage):
    for transf in tfmessage.transforms:
        transf.header.stamp=rospy.Time.now()
    tf_pub.publish(tfmessage)
    
def depth_callback(msg):
	msg.header.stamp = rospy.Time.now()
	depth_pub.publish(msg)

def depth_camera_callback(msg):
	msg.header.stamp = rospy.Time.now()
	depth_camera_pub.publish(msg)
	
def rgb_callback(msg):
	msg.header.stamp = rospy.Time.now()
	rgb_pub.publish(msg)
	
def camera_callback(msg):
	msg.header.stamp = rospy.Time.now()
	camera_pub.publish(msg)
	
def map_callback(msg):
	msg.header.stamp = rospy.Time.now()
	map_pub.publish(msg)

def odom_callback(msg):
	msg.header.stamp = rospy.Time.now()
	odom_pub.publish(msg)

def scan_callback(msg):
	msg.header.stamp = rospy.Time.now()
	scan_pub.publish(msg)

tf_sub = rospy.Subscriber("tf_old",tf.msg.tfMessage,tf_callback)
depth_sub = rospy.Subscriber("camera/depth/image_raw_old",Image,depth_callback)
depth_camera_sub = rospy.Subscriber("camera/depth/camera_info_old", CameraInfo, depth_camera_callback)
rgb_sub = rospy.Subscriber("camera/rgb/image_raw_old",Image, rgb_callback)
camera_sub = rospy.Subscriber("camera/rgb/camera_info_old",CameraInfo, camera_callback)
#map_sub = rospy.Subscriber("/map_old", OccupancyGrid, map_callback)
odom_sub = rospy.Subscriber("/odom_old",Odometry, odom_callback)
scan_sub = rospy.Subscriber("/scan_old", LaserScan, scan_callback)

rospy.spin()
