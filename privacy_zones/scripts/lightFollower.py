#!/usr/bin/env python
import rospy
import os, yaml
from privacy_zones.msg import Transition
from peac_bridge.srv import UpdateControl
import sys

zone_controls = None
update_control_proxy = None

def transition_cb(msg):
    if msg.zone.name in zone_controls:
        control = zone_controls[msg.zone.name][0]
        if msg.action == Transition.ENTER:
            rospy.loginfo('turning %s on' % control['controlId'])
            update_control_proxy(control['controlId'], 100)
        elif msg.action == Transition.EXIT:
            rospy.loginfo('turning %s off' % control['controlId'])
            update_control_proxy(control['controlId'], 0)

if __name__ == '__main__':
    argv = rospy.myargv(sys.argv)
    zone_controls = yaml.load(open(argv[1]))
    rospy.init_node('light_follower')
    rospy.wait_for_service('peac/update_control')
    update_control_proxy = rospy.ServiceProxy('peac/update_control', UpdateControl)
    rospy.Subscriber('transition', Transition, transition_cb)
    rospy.spin()