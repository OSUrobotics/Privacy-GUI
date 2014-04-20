#!/usr/bin/env python

import roslib; roslib.load_manifest('magic_tabletop')

import rospy
import tf
from sensor_msgs.msg import PointCloud2


class Transformer():
    def __init__(self):
        self.target_frame = 'plane'
        self.lis = tf.TransformListener()
        self.pub = rospy.Publisher('/camera/depth_registered/points_tf', PointCloud2)
        self.sub = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.transform_cloud)
        

    def transform_cloud(self, cloud):
        """ Perform transformation. """
        cloud.header.stamp = rospy.Time(0)
        self.lis.waitForTransform(self.target_frame,
                                  cloud.header.frame_id,
                                  rospy.Time(0),
                                  rospy.Duration(3.0))
        cloud_tf = self.lis.transformPointCloud(self.target_frame,
                                                cloud)
        self.pub.publish(cloud_tf)
        

if __name__ == '__main__':

    rospy.init_node('transform_PointCloud2')
    myTransformer = Transformer()
    rospy.spin()
