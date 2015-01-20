#!/usr/bin/env python

from privacy_zones.map_geometry import MapGeometry, Zones
from privacy_zones.msg import Transition
import sys
import yaml
from easy_markers.generator import MarkerGenerator
from visualization_msgs.msg import Marker, MarkerArray
import rospy

active_zones = {}

def transition_cb(msg):
    if msg.action == Transition.ENTER:
        active_zones[msg.zone.name] = [0,0,1,1]
    elif msg.action == Transition.EXIT and msg.zone.name in active_zones:
        del active_zones[msg.zone.name]

if __name__ == '__main__':
    rospy.init_node('zone_viz')
    marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray)
    rospy.Subscriber('transition', Transition, transition_cb)


    zone_file_path, map_info_path = sys.argv[1], sys.argv[2]
    with open(zone_file_path, 'r') as zone_file:
        zones = yaml.load(zone_file)['Zone List']

    geom = MapGeometry(map_info_path)

    z = Zones(zones, geom)

    while not rospy.is_shutdown():
        marker_array = z.to_marker_array(colors=active_zones)
        marker_pub.publish(marker_array)
        rospy.sleep(0.25)