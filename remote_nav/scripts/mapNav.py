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
import sys, getopt
import copy
from os import path

#Qt Bindings
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtOpenGL import *

#Messages for specific functions and goal setting if needed
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalID

import tf
import rospkg
import threading

# provides method for converting MapMetaData yaml to python class
from MapMetaData import *


## Main Window
##^^^^^^^^^^^^^

class Window(QMainWindow): 
	# Window Constructor
	def __init__(self, yaml):

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

		self.meta_data = yaml_to_registered(yaml)
		self.img = path.dirname(yaml) + "/" + self.meta_data.semantic_map
		self.initMapFrame()
		self.setGeometry(300, 300, 350, 250)
		self.setWindowTitle('Robot Map')
		self.show()

	#This function puts together the visuals in the init.  
	def initMapFrame(self):
		self.world=QGraphicsView()
		self.scene=QGraphicsScene()
		self.scene.addPixmap(QPixmap(self.img))


##ROBOT STUFF
##^^^^^^^^^^^
		self.harris = Robot(self.meta_data)

		#feel free to make whatever functions you want. You can also edit the  Robot class

		#For now, this is the robot.  Will change soon. 
		self.scene.addItem(self.harris)
		self.harris.setPoint(0, 0) #This is how you change the position (X, Y)
		self.harris.setTransformOriginPoint(10, 10) #Changes the origin for its own transformations. Maybe make center?
		self.harris.setRotate(0) # This is how you change the rotation (degrees)

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
	#Returns pose of robot in /map frame.
	def _get_robot_pose(self):
		try:
			(trans, rot) = self.listener.lookupTransform("/map","/base_footprint", rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			return None

		goal = PoseStamped()
		goal.header.frame_id = "/map"
		goal.pose.position.x = trans[0]
		goal.pose.position.y = trans[1]
		# print "Robot positin in /map frame: (", goal.pose.position.x, ",", goal.pose.position.y, ")"
		goal.pose.orientation.z = rot[2]
		goal.pose.orientation.w = rot[3]
		return goal
	def quaternion_to_angle(self):
		quaternion = (x,y,z,w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		yaw = euler[2] # yaw gonna make me lose my mind,
		pitch = euler[1] #up in pitch
		roll = euler[0] #up in roll
		yaw = yaw + pi

		newQuat = tf.transformations.quaternion_from_euler(roll,pitch, yaw)

		
	def update_robot_pose(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			current_pose = self._get_robot_pose()
			if current_pose is not None:
				self.harris.setPoint(current_pose.pose.position.x, current_pose.pose.position.y)
				quaternion = (current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w)
				euler = tf.transformations.euler_from_quaternion(quaternion)
				yaw = euler[2] # yaw gonna make me lose my mind,
				yaw = yaw + pi
				self.harris.setRotate(yaw)
				self.scene.update()
			r.sleep()

##ROBOT OBJECT CLASS
##^^^^^^^^^^^^^^^^^^
class Robot(QGraphicsItem):

	angleChanged = pyqtSignal(float)
	robot_width = 0.6   # meters, in the real world

	def __init__(self, meta_data, parent=None):
		super(Robot, self).__init__(parent)

		# Get the relevant information from the yaml
		map_meta_data = meta_data
		self.origin = map_meta_data.origin
		self.resolution = map_meta_data.resolution
		# print "Map Origin: ", self.origin
		self.img_height = map_meta_data.slam_height
		self.transform = parse_transform(map_meta_data.slam_to_semantic[0]['affine'])

		# We use rospack to find the filepath for pr2headUp.png.
		self.rospack = rospkg.RosPack()
		package_path = self.rospack.get_path('remote_nav')
		# Set the image and scale it 
		self.img = QPixmap(package_path + '/images/pr2HeadUp.png')
		self.robot_size = (int)(self.robot_width / self.resolution)
		self.img.scaled(self.robot_size, self.robot_size)

		# Make private variables for the orientation and rotation
		# until we know where the robot is, it will start at image origin
		self.x_pos = 0.0
		self.y_pos = 0.0
		self.rotation = 0.0

#Work in progress paint event (trying to draw the robot as an image. Using a square for now
	def paint(self, painter, option, widget):
		size = self.robot_size
		half_size = size / 2
		painter.drawPixmap(QRect(self.x_pos - half_size, self.y_pos - half_size, size, size), self.img)
		#painter.setRenderHint(QPainter.Antialiasing)
		if self.img.width() > size:
			self.img = self.img.scaled(size, size, Qt.KeepAspectRatio, Qt.SmoothTransformation)

	def boundingRect(self):
		width = 20
		height = 20
		return QRectF(self.x_pos - 10, self.y_pos - 10, width, height)
	
	#Sets the rotation in the Image frame, given real-world rotation
	def setRotate(self, yaw):
		self.rotation = yaw - self.origin[2]
		self.setRotation(self.rotation)
	# Gets the rotation in the image
	def getRotate(self):
		return self.rotation

	# Sets the coordinates for use in the Image frame based on the 
	# real-world coordinates (relative to /map frame)
	def setPoint(self, x, y):
		x = ((- self.origin[0] + x) / self.resolution) 
		y = (self.img_height + ((self.origin[1] - y) / self.resolution)) 
		print "Robot position in SLAM map: (", x, ",", y, ")"
		trans = self.transform
		self.x_pos = (trans[0][0] * x) + (trans[0][1] * y) + trans[0][2]
		self.y_pos = (trans[1][0] * x) + (trans[1][1] * y) + trans[1][2]
		self.x_pos /= 2
		self.y_pos /= 2
		print "Robot position in Image frame: (", self.x_pos, ",", self.y_pos, ")"
		self.setPos(self.x_pos, self.y_pos)
	# gets the position in the image
	def getPoint(self):
		return {'x': self.x_pos, 'y': self.y_pos}

def parse_input(argv):
	usage = argv[0] + " <registered map file>"
	f = ""

	try:
		opts, args = getopt.getopt(argv, "h")
	except getopt.GetoptError as e:
		print usage
		sys.exit(2)

	if '-h' in opts:
		print usage
		sys.exit()

	if len(args) != 2:
		print usage
		sys.exit(2)

	f = args[1]
	return f

## Start the Application
## ^^^^^^^^^^^^^^^^^^^^^
if __name__ == '__main__':
	app = QApplication( sys.argv )

	f = parse_input(sys.argv)
	print f

	rospy.init_node('move')

	mainWindow = Window(f)
	mainWindow.resize( 1000, 1000 )
	mainWindow.show()

	t = threading.Thread(target=mainWindow.update_robot_pose)
	t.daemon = True
	t.start()

	app.exec_()
