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

# provides method for converting MapMetaData yaml to python class
from MapMetaData import *


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

		self.initMapFrame()
		self.setGeometry(300, 300, 350, 250)
		self.setWindowTitle('Robot Map')
		self.show()

	#This function puts together the visuals in the init.  
	def initMapFrame(self):
		self.world=QGraphicsView()
		self.scene=QGraphicsScene()
		self.scene.addPixmap(QPixmap(self.package_path + "/maps/labtest.pgm"))


##ROBOT STUFF
##^^^^^^^^^^^
		self.harris = Robot()
		print ("Rotation is: " )
		print self.harris.getRotate()



		#feel free to make whatever functions you want. You can also edit the  Robot class



		#For now, this is the robot.  Will change soon. 
		self.scene.addItem(self.harris)
		self.harris.setPoint(0, 0) #This is how you change the position (X, Y)
		self.harris.setTransformOriginPoint(10, 10) #Changes the origin for its own transformations. Maybe make center?
		self.harris.setRotation(27) # This is how you change the rotation (degrees)

		#This is how you change the scale of the object.
		#A scale of 0 is a single point
		#A negative scale flips it over the origin (so it will be mirrored)
		#Regular scale is 1.0
		self.harris.setScale(1.0)


#----------end  robot stuff -----------

		self.world.setScene(self.scene)
		self.setCentralWidget(self.world)

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


##ROBOT OBJECT CLASS
##^^^^^^^^^^^^^^^^^^
class Robot(QGraphicsItem):

	angleChanged = pyqtSignal(float)
	robot_width = 0.6 	# meters, in the real world

	def __init__(self, parent=None):
		super(Robot, self).__init__(parent)

		rospack = rospkg.RosPack()
		package_path = rospack.get_path('remote_nav')

		# Get the relevant information from the yaml
		map_meta_data = yaml_to_meta_data(package_path + '/maps/labtest.yaml')
		self.origin = map_meta_data.origin
		self.resolution = map_meta_data.resolution

		# Set the image and scale it 
		self.img = QPixmap(package_path + '/images/pr2HeadUp.png')
		robot_size = (int)(self.robot_width / self.resolution)
		self.img.scaled(robot_size, robot_size)

		# Make private variables for the orientation and rotation
		# until we know where the robot is, it will start at image origin
		self.x_pos = 0.0
		self.y_pos = 0.0
		self.rotation = 0.0
		#set up the Qlabel to be an image of the robot
		#Make private variables for the orientation and rotation
		#also set up sizehint

#Work in progress paint event (trying to draw the robot as an image. Using a square for now
	# def paint(self, painter, option, widget):
	# 	painter = QPainter(self)
	# 	painter.setRenderHint(QPainter.Antialiasing)

	# 	painter.fillRect(event.rect(), self.palette().brush(QPalette.Window))
	# 	painter.setPen(Qt.blue)
	# 	painter.setFont(QFont("Arial", 20))
	# 	painter.drawText(rect(),QAlignCenter, "Qt")
	# 	painter.save()

	# def boundingRect(self):
	# 	width = 20
	# 	height = 20
	# 	return QRectF(QPoint(x_pos, y_pos), QSize(width, height))

	def boundingRect(self):
		penWidth = 1.0
		return QRectF(-10 - penWidth / 2, -10 - penWidth / 2,
			20 + penWidth, 20 + penWidth)

	def paint(self, painter, option, widget):
		painter.drawRoundedRect(-10, -10, 20, 20, 5, 5)
	
	#Sets the rotation in the Image frame, given real-world rotation
	def setRotate(self, yaw):
		self.rotation = yaw + self.origin[2]
	# Gets the rotation in the image
	def getRotate(self):
		return self.rotation

	# Sets the coordinates for use in the Image frame based on the 
	# real-world coordinates (relative to /map frame)
	def setPoint(self, real_x, real_y):
		self.x_pos = (real_x - self.origin[0]) / self.resolution
		self.y_pos = (real_y - self.origin[1]) / self.resolution
		self.setPos(self.x_pos, self.y_pos)
	# gets the position in the image
	def getPoint(self):
		return {'x': self.x_pos, 'y': self.y_pos}
		







## Start the Application
## ^^^^^^^^^^^^^^^^^^^^^
if __name__ == '__main__':
	app = QApplication( sys.argv )
	rospy.init_node('move')

	mainWindow = Window()
	mainWindow.resize( 1000, 1000 )
	mainWindow.show()

	app.exec_()
