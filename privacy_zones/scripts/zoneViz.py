#!/usr/bin/env python

from privacy_zones.map_geometry import MapGeometry, Zones
import sys
import yaml
from easy_markers.generator import MarkerGenerator
from visualization_msgs.msg import Marker, MarkerArray
import rospy

if __name__ == '__main__':

    rospy.init_node('zone_viz')
    marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray)

    zone_file_path, map_info_path = sys.argv[1], sys.argv[2]
    with open(zone_file_path, 'r') as zone_file:
        zones = yaml.load(zone_file)['Zone List']

    geom = MapGeometry(map_info_path)

    z = Zones(zones, geom)
    marker_array = z.to_marker_array()

    while not rospy.is_shutdown():
        marker_pub.publish(marker_array)
        rospy.sleep(0.25)