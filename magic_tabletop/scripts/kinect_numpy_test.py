#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, Image
#from rospy.numpy_msg import numpy_msg
#from rospy_tutorials.msg import Floats
import numpy
import code 
from std_msgs.msg import String

import roslib; roslib.load_manifest('pr2_python')
from pr2_python import conversions
from sensor_msgs.point_cloud2 import read_points


class CloudConverter():
    def __init__(self):
        self.pub = rospy.Publisher('/cloud_done', String)
        self.sub = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.view_cloud)


    def view_cloud(self, cloud):
        cloud_mat = conversions.point_cloud_to_mat(cloud)
        self.pub.publish('Cloud converted!')
        code.interact(local=locals())
    

if __name__ == '__main__':

    rospy.init_node('kinect_numpy_test')

    myCloudConverter = CloudConverter()

    rospy.spin()
