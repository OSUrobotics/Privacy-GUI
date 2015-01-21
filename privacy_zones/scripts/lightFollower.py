#!/usr/bin/env python
import rospy
import os, yaml
from privacy_zones.msg import Transition
from peac_bridge.srv import UpdateControl


zone_controls = yaml.load(open('/home/lazewatd/Dropbox/dev/wheelchair_ws/src/Privacy-GUI/privacy_zones/jolley4_zone_controls.yaml'))
update_control_proxy = None

def transition_cb(msg):
    if msg.zone.name in zone_controls:
        control = zone_controls[msg.zone.name][0]
        if msg.action == Transition.ENTER:
            print 'turning %s on' % control['controlId']
            update_control_proxy(control['controlId'], 1)
        elif msg.action == Transition.EXIT:
            print 'turning %s off' % control['controlId']
            update_control_proxy(control['controlId'], 0)

rospy.init_node('light_follower')
rospy.wait_for_service('update_control')
update_control_proxy = rospy.ServiceProxy('update_control', UpdateControl)
rospy.Subscriber('transition', Transition, transition_cb)
rospy.spin()