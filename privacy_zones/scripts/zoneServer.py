#!/usr/bin/env python

from privacy_zones.map_geometry import MapGeometry
import sys
import yaml
import rospy

class ZoneServer(object):
    def __init__(self, zone_file_path, map_info_path):
        with open(zone_file_path, 'r') as zone_file:
            self.zones = yaml.load(zone_file)['Zone List']

        geom = MapGeometry(map_info_path)

if __name__ == '__main__':

    rospy.init_node('zone_server')