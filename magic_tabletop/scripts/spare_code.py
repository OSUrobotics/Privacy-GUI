
import roslib; roslib.load_manifest('magic_tabletop')

import rospy
import cv
import cv2
from cv_bridge import CvBridge
import tf
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Point, PointStamped, Vector3Stamped, TransformStamped
import numpy
from image_geometry import PinholeCameraModel
from PIL import Image as img
from StringIO import StringIO
from math import sqrt, sin, cos, atan2, floor, pi



class PointFromPixel():
    """ Given a pixel location, find its 3D location in the world """
    def __init__(self, topic_camera_info, topic_depth):
        self.need_camera_info = True
        self.need_depth_image = True
        self.model = PinholeCameraModel()
        rospy.Subscriber(topic_camera_info, CameraInfo, self.callback_camera_info)
        rospy.Subscriber(topic_depth, Image, self.callback_depth_image)

    def callback_camera_info(self, info):
        """ Define Pinhole Camera Model parameters using camera info msg """
        if self.need_camera_info:
            rospy.loginfo('Got camera info!')
            self.model.fromCameraInfo(info)  # define model params
            self.frame = info.header.frame_id
            self.need_camera_info = False

    def callback_depth_image(self, depth_image):
        """ Get depth at chosen pixel using depth image """
        if self.need_depth_image:
            rospy.loginfo('Got depth image!')
            self.depth = img.fromstring("F", (depth_image.width, depth_image.height), depth_image.data)
            self.need_depth_image = False

    def calculate_3d_point(self, pixel):
        """ Project ray through chosen pixel, then use pixel depth to get 3d point """
        lookup = self.depth.load()
        depth = lookup[pixel[0], pixel[1]]  # lookup pixel in depth image
        ray = self.model.projectPixelTo3dRay(tuple(pixel))  # get 3d ray of unit length through desired pixel
        ray_z = [el / ray[2] for el in ray]  # normalize the ray so its Z-component equals 1.0
        pt = [el * depth for el in ray_z]  # multiply the ray by the depth; its Z-component should now equal the depth value
        point = PointStamped()
        point.header.frame_id = self.frame
        point.point.x = pt[0]
        point.point.y = pt[1]
        point.point.z = pt[2]
        return point

    def ray_plane_intersection(self, pixel, plane):
        """ Given plane parameters [a, b, c, d] as in ax+by+cz+d=0,
        finds intersection of 3D ray with the plane. """ 
        ray = self.model.projectPixelTo3dRay(tuple(pixel))  # get 3d ray of unit length through desired pixel
        scale = -plane[3] / (plane[0]*ray[0] + plane[1]*ray[1] + plane[2]*ray[2])

        point = PointStamped()
        point.header.frame_id = self.frame
        point.point.x = ray[0] * scale
        point.point.y = ray[1] * scale
        point.point.z = ray[2] * scale
        return point


class PlaneFinder():
    """ Gets plane parameters for filtered point cloud. """
    def __init__(self):
        pass

    def find_plane_from_cloud(self, cloud):
        """ """
        # Find plane from point cloud
        fit_plane = rospy.ServiceProxy('fit_plane', FitPlane)
        request = FitPlaneRequest()
        request.cloud = cloud
        response = fit_plane(request)
        coeffs_list = [response.a, response.b, response.c, response.d]
        normal_length = sqrt(sum([coeff**2 for coeff in coeffs_list[:3]]))
        self.normal = [coeff/normal_length for coeff in coeffs_list]  # includes all four coefficients

    def tf_from_plane(self, point, normal):
        """ Finds TF frame based on plane normal and a point. INPUTS: reference position (PointStamped), normal [x, y, z] """
        frame = TransformStamped()
        frame.header.frame_id = point.header.frame_id
        frame.transform.translation.x = point.point.x
        frame.transform.translation.y = point.point.y
        frame.transform.translation.z = point.point.z
        
        normal = -1 * numpy.asarray(normal)  # convert knob normal vector to numpy array
        k = numpy.array([0, 0, 1])  # z-axis 
        q = self.quaternion_from_directions(normal, k)  # get quaternion to rotate normal to z-axis
        
        frame.transform.rotation.x = q[0]
        frame.transform.rotation.y = q[1]
        frame.transform.rotation.z = q[2]
        frame.transform.rotation.w = q[3]

        return frame

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


    ####### Convert region to 3D ############
    point_from_pixel = PointFromPixel('/head_mount_kinect/depth_registered/camera_info', 
                                      '/head_mount_kinect/depth_registered/image_rect')
    while point_from_pixel.need_camera_info or point_from_pixel.need_depth_image:
        rospy.sleep(0.01)
    region = [[], []]
    for i in range(len(grab_reg.region)):
        for j in range(len(grab_reg.region[i])):
            region[i].append(point_from_pixel.calculate_3d_point(grab_reg.region[i][j]))  # get 3d point that corresponds to pixel

    ####### Filter cloud based on region ########
    cloud_filter = CloudFilterByRegion(region, '/head_mount_kinect/depth_registered/points')
    while cloud_filter.need_pointcloud:
        rospy.sleep(0.01)
    cloud_filter.filter_cloud_by_region()

    frame = plane.tf_from_plane(region[0][0], plane.normal[:3])


