#!/usr/bin/env python

from privacy_zones.map_geometry import MapGeometry
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

    gen = MarkerGenerator()
    gen.ns = '/map_zones'
    gen.type = Marker.LINE_STRIP
    gen.scale = [0.3]*3
    gen.frame_id = 'map'
    gen.color = [1,0,0,1]

    marker_array = MarkerArray()

    for zone in zones:
        world_points = [geom.px_to_world_coords((point['x'], point['y'])).tolist() for point in zone['Points']]
        world_points = [(p[0],p[1],0) for p in world_points]
        world_points.append(world_points[0])
        marker = gen.marker(points=world_points)
        marker_array.markers.append(marker)

    while not rospy.is_shutdown():
        marker_pub.publish(marker_array)
        rospy.sleep(0.25)