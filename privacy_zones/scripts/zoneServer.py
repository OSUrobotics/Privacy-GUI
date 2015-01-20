#!/usr/bin/env python

from privacy_zones.map_geometry import MapGeometry, Zones
from privacy_zones.msg import Transition
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from shapely.geometry import Point
import sys
import yaml
import rospy
import tf

class ZoneServer(object):
    def __init__(self, zone_file_path, map_info_path):
        with open(zone_file_path, 'r') as zone_file:
            zones = yaml.load(zone_file)['Zone List']

        self.geom = MapGeometry(map_info_path)
        self.zones = Zones(zones, self.geom)
        self.current_zones = []
        self.previous_zones = []
        self.last_transition_time = rospy.Time(0)

        rospy.Subscriber('pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.transition_pub = rospy.Publisher('transition', Transition)

        self.tfl = tf.TransformListener()

    def pose_cb(self, msg):
        msg_trans = self.tfl.transformPose('map', msg)
        pt = Point(msg_trans.pose.position.x, msg_trans.pose.position.y)
        within_zones = self.zones.in_which(pt)
        if within_zones:
            self.previous_zones = self.current_zones
            self.current_zones = within_zones
            self.check_transition()

    def odom_cb(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.pose_cb(pose)

    def check_transition(self):
        if self.previous_zones and self.current_zones:
            entered_zones = set(self.current_zones) - set(self.previous_zones)
            exited_zones = set(self.previous_zones) - set(self.current_zones)
            for zone in entered_zones:
                transition_msg = Transition(action=Transition.ENTER, zone=zone.to_msg())
                self.transition_pub.publish(transition_msg)
            for zone in exited_zones:
                transition_msg = Transition(action=Transition.EXIT, zone=zone.to_msg())
                self.transition_pub.publish(transition_msg)

if __name__ == '__main__':
    rospy.init_node('zone_server')
    server = ZoneServer(sys.argv[1], sys.argv[2])
    rospy.spin()