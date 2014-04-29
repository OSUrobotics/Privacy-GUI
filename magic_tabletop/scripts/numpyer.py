#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, PointCloud2
import numpy
import cv
from cv_bridge import CvBridge

import code # FOR TESTING

import roslib; roslib.load_manifest('pr2_python')
from pr2_python import pointclouds


class CloudNumpyer():
    def __init__(self, topic):
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Float32MultiArray, self.array_callback)
        self.pub = rospy.Publisher(topic+'_image', Image)

    def array_callback(self, array_in):
        """ Convert unpacked point cloud (Float32MultiArray msg) to numpy array. """
        self.array = numpy.asarray(array_in.data)  # convert to numpy
        self.array = numpy.reshape(self.array, (480, 640, 6))  # reshape to (480, 640, 6)
        rospy.loginfo('Converted a cloud to numpy!')

        # DEMO: Turn everything >2m away to black.
        #code.interact(local=locals())
        mask = self.array[:, :, 2] > 2
        for i in [3, 4, 5]: self.array[:, :, i][mask] = 0

        image_np = self.array[:, :, 3:].astype('uint8')
        image_cv = cv.fromarray(image_np)
        image_msg = self.bridge.cv_to_imgmsg(image_cv, encoding='rgb8')
        self.pub.publish(image_msg)
        

class ImageNumpyer():
    def __init__(self, topic):
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.image_callback)
        
    def image_callback(self, image_in):
        """ Convert Image msg to numpy array. """
        image_cv = self.bridge.imgmsg_to_cv(image_in)
        self.image = numpy.asarray(image_cv)
        rospy.loginfo('Converted an image to numpy!')

class Cloud2Numpyer():
    def __init__(self, topic):
        self.bridge = CvBridge()
        rospy.Subscriber(topic, PointCloud2, self.cloud_callback)
        self.pub = rospy.Publisher(topic+'_image', Image)

    def cloud_callback(self, cloud_in):
        """ Convert PointCloud2 msg to numpy array. """
        cloud_full = pointclouds.pointcloud2_to_array(cloud_in)
        cloud_xyz = pointclouds.pointcloud2_to_xyz_array(cloud_in, remove_nans=False)
        
        

if __name__ == '__main__':

    rospy.init_node('numpyize_PointCloud2')

    #cloudNumpyer = CloudNumpyer('/camera/depth_registered/points_unpacked')

    #imageNumpyer = ImageNumpyer('/camera/rgb/image_color')

    cloud2Numpyer = Cloud2Numpyer('/camera/depth_registered/points')

    rospy.spin()
