#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

#This class is for controlling the position.x and orientation of the robot along some axis.
class Goal():
	def __init__(self, length, pose, forward=True):
		#Length of the hallway.
		self._length = length
		#Keep a constant pose that we update.
		self._navGoal = pose
		#Are we going forward along our track?
		self._forward = forward
	def getPose(self):
		return self._navGoal
	#Increment the distance from the "/start" frame by some increment.
	def inc(self, increment=0.1):
		self._navGoal.header.stamp = rospy.Time.now()
		if 	self._forward:
			if (self._navGoal.pose.position.x + increment < self._length):
				self._navGoal.pose.position.x += increment
		else:
			if (self._navGoal.pose.position.x - increment > 0):
				self._navGoal.pose.position.x = self._navGoal.pose.position.x - increment
		return self._navGoal
	#Reverse the orientation 
	def reverseOrientation(self):
		self._navGoal.header.stamp = rospy.Time.now()
		self._forward = not self._forward
		return self._navGoals
