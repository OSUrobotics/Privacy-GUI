#!/usr/bin/env python

import roslib; roslib.load_manifest('magic_tabletop')

import rospy
import tf
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PointStamped, TransformStamped
import numpy
from math import sqrt, sin, cos, atan2, floor, pi

from magic_tabletop.srv import *


class PlanePublisher():
    def __init__(self):
        # Set up service and TF broadcaster
        self.need_plane = True
        self.br = tf.TransformBroadcaster()
        self.srv = rospy.Service('publish_plane_tf', PublishPlaneTF, self.get_plane)

        
    def get_plane(self, coeffs):
        """ Grab coefficients of detected plane. """
        self.coeffs = coeffs
        self.need_plane = False
        rospy.loginfo('Got plane coefficients: {0}'.format(self.coeffs))
        return PublishPlaneTFResponse()


    def tf_from_plane(self, point):
        """ Finds TF frame based on plane normal and a point. INPUTS: reference position (PointStamped) """
        frame = TransformStamped()
        frame.header.frame_id = point.header.frame_id
        frame.transform.translation.x = point.point.x
        frame.transform.translation.y = point.point.y
        frame.transform.translation.z = point.point.z
        
        normal = [-1 * self.coeffs.a, -1 * self.coeffs.b, -1 * self.coeffs.c]
        normal = -1 * numpy.asarray(normal)  # convert knob normal vector to numpy array
        k = numpy.array([0, 0, 1])  # z-axis 
        q = self.quaternion_from_directions(normal, k)  # get quaternion to rotate normal to z-axis
        
        frame.transform.rotation.x = q[0]
        frame.transform.rotation.y = q[1]
        frame.transform.rotation.z = q[2]
        frame.transform.rotation.w = q[3]

        self.frame = frame
        rospy.loginfo('Created TF frame aligned with plane normal!')


    def quaternion_from_directions(self, v_from, v_to):
        """ Given two vectors as lists, constructs a numpy quaternion of form [x, y, z, w] 
        that rotates from first input 'v_from' to second input 'v_to' """
        v_from = numpy.asarray(v_from)  # convert list to numpy array
        v_from /= numpy.linalg.norm(v_from)  # normalize

        v_to = numpy.asarray(v_to)  # convert list to numpy array
        v_to /= numpy.linalg.norm(v_to)  # normalize

        u = numpy.cross(v_to, v_from)  # axis of rotation

        theta_sin = numpy.linalg.norm(u)
        theta_cos = numpy.dot(v_to, v_from)
        theta = atan2(theta_sin, theta_cos)  # angle of rotation 
        u /= numpy.linalg.norm(u)  # axis of rotation, normalized     
        
        # Convert to quaternion notation 
        w = cos(0.5 * theta)
        xyz = sin(0.5 * theta) * u
        
        return numpy.array([xyz[0], xyz[1], xyz[2], w])  # quaternion in numpy array format


    def publish_tf(self):
        """ Publishes TF frame continuously. """
        transform = self.frame.transform  # shortcut
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.br.sendTransform((transform.translation.x, transform.translation.y, transform.translation.z),
                                  (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w),
                                  rospy.Time.now(),
                                  "plane",
                                  "camera_rgb_optical_frame")
            r.sleep()
        
    
if __name__ == '__main__':

    rospy.init_node('publish_plane_tf')

    pub = PlanePublisher()

    while pub.need_plane:
        rospy.sleep(0.01)
        
    # Make point for TF 
    # (plane equation: ax + by + cz + d = 0)
    # (solving for z: z = [-d - ax - by] / c)
    pt = PointStamped()  # x = y = z = 0 
    pt.header.frame_id = 'camera_rgb_optical_frame'
    rospy.loginfo('WARNING: assuming the plane is in frame camera_rgb_optical_frame')
    pt.point.z = (-1 * pub.coeffs.d - pub.coeffs.a * pt.point.x - pub.coeffs.b * pt.point.y) / pub.coeffs.c
    rospy.loginfo('Gaze vector intersects plane at {0} meters.'.format(pt.point.z))
    
    # Make TF
    pub.tf_from_plane(pt)
    
    # Publish TF continuously
    pub.publish_tf()
