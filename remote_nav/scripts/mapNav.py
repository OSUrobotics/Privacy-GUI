#!/usr/bin/env python

#Hey Penn, this file here should be enough to get everything going minimally.


## Imports
## ^^^^^^^
# Standard ROS imports
import roslib; roslib.load_manifest('rviz_python_tutorial')
import rospy

#math because Math
from math import *

#stuff for sys.argv
import sys
import copy

#Qt Bindings
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtOpenGL import *

#Messages for specific functions and goal setting if needed
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalID

import tf
import rospkg


## Main Window
##^^^^^^^^^^^^^

class Window(QMainWindow): 
	# Window Constructor
	def __init__(self):

		QWidget.__init__(self)
		# We use rospack to find the filepath for remote_nav.
		self.rospack = rospkg.RosPack()
		self.package_path = self.rospack.get_path('remote_nav')

		#A publisher to literally tell dis bisnatch to cancel all goals.
		self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID)
		self.listener = tf.TransformListener()
		#Gets track length of ID start track
		self.track_length = rospy.get_param('remote_nav/track_length', 5.0)
		self.robot_frame = rospy.get_param('remote_nav/robot_frame', "/base_footprint")

		self.initUI()

	#This function puts together the visuals in the init.  
	def initUI(self):
		self.world=QGraphicsView()
		self.scene=QGraphicsScene()




		#Add the map background. You can change the path here if you want.
		self.scene.addPixmap(QPixmap(self.package_path + "/maps/labtest.pgm"))
		self.scene.addEllipse(0, 0, 20, 20)
		self.world.setScene(self.scene)
		self.setCentralWidget(self.world)
		self.setGeometry(300, 300, 350, 250)
		self.setWindowTitle('Robot Map')
		self.show()

	def closeEvent(self, event):
		reply = QMessageBox.question(self, 'Message',
		"Are you sure to quit?", QMessageBox.Yes | 
		QMessageBox.No, QMessageBox.No)

		if reply == QMessageBox.Yes:
			event.accept()
		else:
			event.ignore()

#PRIVATE FUNCTIONS
#^^^^^^^^^^^^^^^^
	# Sends a nav goal to the bot. This is like sending it a position in space to go.
	# If it is necessary to transform to the /map frame, specify so with transform=True
	def _send_nav_goal(self, pose):
		self.nav_pub.publish(pose)

	#Returns transform of robot relative to /start pose.
	def _get_pose_from_start(self):
		(trans, rot) = self.listener.lookupTransform("/start", self.robot_frame, rospy.Time(0))
		start_trans = PoseStamped()
		start_trans.header.frame_id = "/start"
		start_trans.header.stamp = rospy.Time.now()
		start_trans.pose.position.x = trans[0]
		start_trans.pose.position.y = trans[1]
		start_trans.pose.position.z = trans[2]
		start_trans.pose.orientation.x = rot[0]
		start_trans.pose.orientation.y = rot[1]
		start_trans.pose.orientation.z = rot[2]
		start_trans.pose.orientation.w = rot[3]
		return start_trans
	#Get start frame's pose with parent frame /map.
	def _get_start_pose(self):
		(trans, rot) = self.listener.lookupTransform("/map","/start", rospy.Time(0))
		goal = PoseStamped()
		goal.header.frame_id = "/map"
		goal.pose.position.x = trans[0]
		goal.pose.position.y = trans[1]

		goal.pose.orientation.z = rot[2]
		goal.pose.orientation.w = rot[3]
		return goal

# The  Robot Object, THIS IS WHAT YOU EDIT :D
class Robot(QLabel):

	angleChanged = pyqtSignal(float)
	rotation = 0.0
	x_pos = 0.0
	y_pos = 0.0

	def __init__(self, parent=None):
		super(Robot, self).__init__(parent)
		#set up the Qlabel to be an image of the robot
		#Make private variables for the orientation and rotation
		#also set up sizehint
	def paintEvent(self, event):
		painter = QPainter()
		painter.setRenderHint(QPainter.Antialiasing)

		painter.fillRect(event.rect(), self.palette().brush(QPalette.Window))
		painter.setPen(Qt.blue)
		painter.setFont(QFont("Arial", 20))
		painter.drawText(rect(),QAlignCenter, "Qt")
		painter.save()
	
	#Sets the rotation in the QWidget frame (not the world map)	
	def setRotate(self, yaw):
		rotation = yaw

	def getRotate(self):
		return rotation

	#Sets the coordinates for use in the QWidget frame (not the world map)
	def setPoint(self, x, y):
		x_pos = x
		y_pos = y
	def getPoint(self):
		return {'x':x_pos, 'y': y_pos}
		







## Start the Application
## ^^^^^^^^^^^^^^^^^^^^^
if __name__ == '__main__':
	app = QApplication( sys.argv )
	rospy.init_node('move')

	mainWindow = Window()
	mainWindow.resize( 1000, 500 )
	mainWindow.show()

	app.exec_()
