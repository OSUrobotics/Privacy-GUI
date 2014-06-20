#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
import cv, cv2
import numpy
import tf


from rosCV import rosCV as rcv

class turtleViz():
    def __init__(self, topic):
        self.camModel = PinholeCameraModel()
        self.listener = tf.TransformListener()
        self.rcv = rcv()
        rospy.Subscriber(topic, Image, self.image_callback)

        rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.camera_callback)
        self.haveCamera = False
        self.pub = rospy.Publisher('turtleVision', Image)

    def camera_callback(self, cameraInfo):
        if not self.haveCamera:
        #we update our camModel with the cameraInfo to work some magic.
            self.camModel.fromCameraInfo(cameraInfo)
            self.haveCamera = True

    def image_callback(self, image_in):
        # Import and convert
        image_cv2 = self.rcv.toCv2(image_in)
        try:
            (trans, rot) = self.listener.lookupTransform('/camera_rgb_optical_frame', '/test2', rospy.Time(0))
            position = (int(trans[0]), int(trans[1]), int(trans[2]))
            projected = self.camModel.project3dToPixel(position)
            rospy.logdebug(projected)
            x = int(projected[0])
            y = int(projected[1])

            image_cv2 = self.rcv.redact(image_cv2, (x,y), (x+50,y+50))
            # Convert back to ROS Image msg
            image_out = self.rcv.toRos(image_cv2)
            self.pub.publish(image_out)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, AttributeError):
        # except ():
            # rospy.logdebug("transform failed")
            pass
        self.rcv.imshow(image_cv2)


if __name__ == '__main__':

    rospy.init_node('turtleVision', log_level=rospy.DEBUG)

    tv = turtleViz('/camera/rgb/image_color_repub')

    rospy.spin()
