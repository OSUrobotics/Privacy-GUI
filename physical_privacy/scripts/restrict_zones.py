#!/usr/bin/env python

# ROS data types
from physical_privacy.srv import RestrictZones
# ROS Libraries
import rospy
# External Libraries


class ZoneRestrictor():
	"""Class that creates a privacy restricted costmap from privacy zones"""
	def __init__(self, arg):
		self.arg = arg
		

if __name__ == "__main__":
	pass