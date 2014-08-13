#!/usr/bin/env python
import rospy
import yaml
import pprint
from geometry_msgs.msg import Polygon, Point32

if __name__ == '__main__':
    rospy.wait_for_service('add_two_ints')
    # We use rospack to find the filepath for physical_privacy, lest the user does not roslaunch.
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('physical_privacy')

    #Our zones are stored in a .yaml file.
    zones_file = rospy.get_param('priv_nav/zones', package_path + "/zones/test.yaml")
    with open(zones_file) as f:
	    zones = yaml.safe_load(f)

    for zone in zones['Zone List']:
    	polygon = Polygon()
        for point in zone['Points']:
	        new_point = Point32()
	        new_point.x = point['x'] 
	        new_point.y = point['y']
	        polygon.points.append(new_point)
	#some service call goes here.
    rospy.spin()



