#!/usr/bin/env python
#
# Makes stuff higher than the floor disappear. Steps:
# 1) Grabs a Kinect PointCloud2 msg
# 2) Finds the dominant plane -- the floor, if the Kinect is pointed down
# 3) Projects all points above that (floor) plane into the floor
# 4) Paints those points a different color
# 5) Converts back to an Image msg, which is displayed to the user as video

import rospy
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import cv, cv2
import numpy

from magic_tabletop.srv import *

class PlaneAxisHandler():
    def __init__(self, topic):
        self.need_cloud = True
        rospy.Subscriber(topic, PointCloud2, self.cloud_callback)

    def cloud_callback(self, cloud):
        """ Get point cloud to which we're subscribed. """
        if self.need_cloud:
            rospy.loginfo('Got a cloud.')
            coeffs = self.find_plane(cloud)
            self.publish_plane(coeffs)
            self.need_cloud = False

    def find_plane(self, cloud):
        """ Get plane coefficients for dominant plane in point cloud. """
        fit_plane = rospy.ServiceProxy('fit_plane', FitPlane)
        fit_plane.wait_for_service()

        req = FitPlaneRequest()
        req.cloud = cloud

        res = fit_plane(req)  # call the service
        rospy.loginfo('Got plane coefficients: ')
        rospy.loginfo(res)
        
        return res

    def publish_plane(self, coeffs):
        """ Send plane coefficients off to be published as a TF frame. """
        publish_coeffs = rospy.ServiceProxy('publish_plane_tf', PublishPlaneTF)
        publish_coeffs.wait_for_service()

        req = PublishPlaneTFRequest()
        req.a = coeffs.a
        req.b = coeffs.b
        req.c = coeffs.c
        req.d = coeffs.d
        
        res = publish_coeffs(req)  # call the service
        rospy.loginfo('Plane coefficients sent off for publishing.')

        return 
        
            

class CloudFilter():
    def __init__(self, topic):
        self.bridge = CvBridge()
        rospy.Subscriber(topic, PointCloud2, self.cloud_callback)
        self.pub_cloud = rospy.Publisher(topic+'_filled', PointCloud2)
        self.pub_image = rospy.Publisher('/camera/rgb/image_mask', Image)

    def cloud_callback(self, cloud):
        """ Get point cloud to which we're subscribed. """
        cloud_tf = self.transform_cloud(cloud)
        filter_result = self.filter_cloud(cloud_tf)
        self.pub_cloud.publish(filter_result.cloud_out)

        # Convert image to black and white
        filter_result.image_out
        image_cv = self.bridge.imgmsg_to_cv(filter_result.image_out, 'bgr8')
        image_cv2 = numpy.array(image_cv, dtype=numpy.uint8)
        green = numpy.logical_and(image_cv2[:, :, 0] == 0,
                                  image_cv2[:, :, 1] == 255)
        green = numpy.logical_and(green,
                                  image_cv2[:, :, 2] == 0)
        for dim in range(3): image_cv2[:, :, dim] = numpy.uint8(green)*255
        image_cv = cv.fromarray(image_cv2)
        image_out_bw = self.bridge.cv_to_imgmsg(image_cv, 'bgr8')
        self.pub_image.publish(image_out_bw)

    def transform_cloud(self, cloud):
        """ Transform cloud to plane TF. """
        transform_cloud = rospy.ServiceProxy('transform_PointCloud2', TransformPointCloud2)
        transform_cloud.wait_for_service()

        req = TransformPointCloud2Request()
        req.cloud_in = cloud
        req.frame_new = 'plane'

        res = transform_cloud(req)  # call the service
        
        return res.cloud_out
        
    def filter_cloud(self, cloud):
        """ Filter points with Z > 0 from point cloud. """
        fill_plane = rospy.ServiceProxy('fill_in_plane', FillInPlane)
        fill_plane.wait_for_service()

        req = FillInPlaneRequest()
        req.cloud_in = cloud
        req.top = 0.10
        req.bottom = -0.03
        
        res = fill_plane(req)  # call the service
        
        return res



if __name__ == '__main__':

    rospy.init_node('magic_floor')

    planeAxisHandler = PlaneAxisHandler('/camera/depth_registered/points')
    cloudFilter = CloudFilter('/camera/depth_registered/points')

    rospy.spin()
