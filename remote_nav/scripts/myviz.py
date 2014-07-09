#!/usr/bin/env python

## Imports
## ^^^^^^^

## First we start with the standard ros Python import line:
import roslib; roslib.load_manifest('rviz_python_tutorial')
import rospy
from math import *
## Then load sys to get sys.argv.
import sys
import copy

## Next import all the Qt bindings into the current namespace, for
## convenience. This uses the "python_qt_binding" package which hides
## differences between PyQt and PySide, and works if at least one of
## the two is installed. The RViz Python bindings use
## python_qt_binding internally, so you should use it here as well.
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

#Get moving!
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from goal import Goal

## Finally import the RViz bindings themselves.
import rviz
import tf

import rospkg

## The MyViz class is the main container widget.
class MyViz( QWidget ):

	## MyViz Constructor
	def __init__(self):

		QWidget.__init__(self)
	#The visualizer
		self.frame = rviz.VisualizationFrame()
		self.frame.setSplashPath( "" )
		self.frame.initialize()

	## The reader reads config file data into the config object.
		## VisualizationFrame reads its data from the config object.
		reader = rviz.YamlConfigReader()
		config = rviz.Config()

		# We use rospack to find the filepath for remote_nav.
		rospack = rospkg.RosPack()
		package_path = rospack.get_path('remote_nav')
		print package_path
		#Now you can grab this filepath from either roslaunch remote_nav myviz and using the launch file or just rosrun.
		config_file = rospy.get_param('remote_nav/rviz_config', package_path + "/config/map_and_img.rviz")
		reader.readFile( config, config_file )
		self.frame.load( config )

		self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )
		self.setWindowIcon(QIcon(package_path +'/images/icon.png'))

	#The twist commands
		self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist) 
		self.zero_cmd_sent = False

	#For sending nav goals.
		self.nav_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)
		self.listener = tf.TransformListener()

	#Subscribe to initialpose to find our start position
		rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initialpose_callback)
		#Is the robot facing forward along our track?
		self.isForward = True
	

	#Disable unneeded views and more visualization setup
		self.frame.setMenuBar( None )
		self.frame.setStatusBar( None )
		self.frame.setHideButtonVisibility( False )
		self.manager = self.frame.getManager()
		self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
		
	# ##--LAYOUT--
	# 	layout = QVBoxLayout()
	# 	layout.addWidget( self.frame )
		layout = QGridLayout()
		layout.setSpacing(10)

		layout.addWidget(self.frame, 1, 0, 4, 3)
		
	 	h_layout = QHBoxLayout()
		
	#Buttons and attached commands
		# 1. Create Button
		# 2. Connect Signal to Slot
		# 3. Add to layout

		self.stop_button = QPushButton( "STOP" )
		self.stop_button.clicked.connect( self.onStopButtonClick )
		self.stop_button.setToolTip('Press this to immediately <b>STOP</b> the robot')
		self.stop_button.setStyleSheet("background-color: #700000 ; font-weight: bold; color: white")
		layout.addWidget( self.stop_button, 5, 1 )

		debug_button = QPushButton( "Reset Position" )
		debug_button.clicked.connect( self.onDebugButtonClick )
		debug_button.setToolTip('Set a navigation goal at the starting point')
		h_layout.addWidget( debug_button )

		reset_dir_btn = QPushButton( "Reset Orientation" )
		reset_dir_btn.clicked.connect( self.onResetDirButtonClick )
		reset_dir_btn.setToolTip('Reset to the original orientation')
		h_layout.addWidget( reset_dir_btn )
		

		self.fwd_button = PicButton(QPixmap(package_path + "/images/forward.png"))
		self.fwd_button = QPushButton("Move Forward")
		self.fwd_button.pressed.connect( self.onFwdPress )
		self.fwd_button.setToolTip('While held, the robot will move forward')
		layout.addWidget( self.fwd_button, 4, 1 )
		# layout.setAlignment(self.fwd_button2, Qt.AlignHCenter)

		turn_button = QPushButton( "Turn Around[ALEX DEBUG - Nav Goals]" )
		turn_button.clicked.connect( self.onTurnButtonClick )
		turn_button.setToolTip('The robot will turn around 180 degrees')
		h_layout.addWidget( turn_button )

		# turn_twist_button = QPushButton( "Turn Around[ALEX DEBUG - Twist message]" )
		# # turn_button.clicked.connect( self.onTurnButtonClick )
		# turn_twist_button.clicked.connect( self.onTurnTwistButtonClick)
		# turn_twist_button.setToolTip('The robot will turn around 180 degrees')
		# h_layout.addWidget( turn_twist_button )

		look_left_btn = PicButton(QPixmap(package_path + "/images/turn_left.png"))
		layout.addWidget(look_left_btn, 2, 0)
		layout.setAlignment(look_left_btn, Qt.AlignLeft)

		look_right_btn = PicButton(QPixmap(package_path + "/images/turn_right.png"))
		layout.addWidget(look_right_btn, 2, 2)
		layout.setAlignment(look_right_btn, Qt.AlignRight)
		
	#Finalizing layout and placing components
		layout.addLayout( h_layout, 6, 0, 1, 3 )	
		self.setLayout( layout )




	## Handle GUI events
	## ^^^^^^^^^^^^^^^^^

	def switchToView( self, view_name ):
		view_man = self.manager.getViewManager()
		for i in range( view_man.getNumViews() ):
			if view_man.getViewAt( i ).getName() == view_name:
				view_man.setCurrentFrom( view_man.getViewAt( i ))
				return
		print( "Did not find view named %s." % view_name )

	# BUTTON CALLBACKS
	# ^^^^^^^^^^^^^^^^
	def onFwdPress(self):
		self.moveNav(0.35)


	def onDebugButtonClick(self):
	#Tells robot to return to home base. Alex, you can continue editing here. 
		self.start.header.stamp = rospy.Time.now()
		self._send_nav_goal(self.start)

	def onStopButtonClick(self):
		QApplication.processEvents()
		goal = self._get_current_goal()
		self._send_nav_goal(goal)
		# Needs to interrupt the turnAround and navTurnAround functions

#
	def onTurnButtonClick(self):
		if self.isForward:
			self.faceBackward()
		else:
			self.faceForward()
	def onTurnTwistButtonClick(self):
		self.turnAround()

	def onResetDirButtonClick(self):
		self.faceForward()

	## NAVIGATION FUNCTIONS
	## ^^^^^^^^^^^^^^^^^^^^

	#Initialize our goal as on the start frame.
	def initialpose_callback(self, initialpose):

		self.start = PoseStamped()
		self.start.header.frame_id = "/map"
		self.start.header.stamp = rospy.Time.now()

		self.start.pose.position = copy.copy(initialpose.pose.pose.position)
		self.start.pose.orientation = copy.copy(initialpose.pose.pose.orientation)

	#Rotate the robot exactly 180 degrees with a twist command
	def turnAround(self):
		command = Twist()
		command.angular.z = 0.5
		now = rospy.Time.now()
		r = rospy.Rate(50) 
		while rospy.Time.now() - now <= rospy.Duration(2*pi):
			QApplication.processEvents()
			print (rospy.Time.now() - now)
		#An attempt to stop it while it is in motion
			if self.stop_button.isDown():
				command.angular.z = 0.0
				break
			self.pub.publish(command)
			r.sleep()
		command.angular.z = 0.0
		self.pub.publish(command)
			

	#Face forward along our track.
	def faceForward(self):
		self.isForward = True
		goal = self._get_current_goal()
		self._send_nav_goal(goal)
		print ("Now facing forward")
	#Face 180 degrees from the forward position.
	def faceBackward(self):
		now = rospy.Time.now()

		#Grab where we currently are.
		self.isForward = False
		goal = self._get_current_goal()
		self._send_nav_goal(goal)
		print ("Now facing backward.")
		

	# #Moves the robot at a given max velocity whenever the forward button is pressed
	# #It still works while the button is held down
	# def moveWhilePressed(self, velocity):
	# 	now = rospy.get_time()
	# 	#Speed up until max velocity
	# 	while rospy.get_time() - now < 2:
	# 		QApplication.processEvents()
	# 		x = rospy.get_time() - now            
	# 		xVel = tanh(x) * velocity
	# 		self._send_twist(xVel)
	# 		if self.stop_button.isDown():
	# 			break
	# 		if not self.fwd_button.isDown():
	# 			break
	# 	#Continue at max while pressed
	# 	while self.fwd_button.isDown():
	# 		QApplication.processEvents()
	# 		if self.stop_button.isDown():
	# 			break
	# 		xVel = velocity
	# 		self._send_twist(xVel)

	# 	#Slow down on button release
	# 	now = rospy.get_time()
	# 	while rospy.get_time() - now < 2:			
	# 		QApplication.processEvents()
	# 		if self.stop_button.isDown():
	# 			break
	# 		x = 2 - (rospy.get_time() - now) 
	# 		xVel = tanh(x) * velocity
	# 		self._send_twist(xVel)
	# 	#Check orientation
	# 	#Realign orientation

	# # Give the turtlebot a distance to travel and a velocity and it follows the command + slows down accordingly as it reaches its destination.
	# def moveAhead(self, distance, velocity):
	# 	# By what facter we scale/lengthen the tanh function.
	# 	s = 1
	# 	# Integrate the x function below and set it equal to distance to find movingTime.
	# 	movingTime = 1.0/s * acosh( exp( s/velocity * distance ) )
	# 	now = rospy.get_time()
	# 	while rospy.get_time() - now < movingTime:
	# 		QApplication.processEvents()
	# 		x = movingTime - (rospy.get_time() - now)            
	# 		xVel = tanh(s * x) * velocity
	# 		self._send_twist(xVel)

	#Moves ahead via nav goals while the button is pressed.
	def moveNav(self, dist):

		# Keep track of how far we've travelled in order to only send new nav goals when need be. 
		# How far we've travelled since last nav goal sent.
		travelled = 0.0
		#oldX indicates our first nav x position.
		goal = self._get_current_goal()
		oldX = goal.pose.position.x
		
		#If we say "travel 1 meter" the robot will probably just travel ~0.9 meters. This trys to account for that by grabbing the x, y tolerance.
		tolerance = rospy.get_param("/move_base/TrajectoryPlannerROS/xy_goal_tolerance", "0.25")
		
		i = 0
		#give an initial command to go.
		if (self.isForward):
			goal.pose.position.x += dist
		else:
			goal.pose.position.x -= dist
		self._send_nav_goal(goal)

		while self.fwd_button.isDown():
			QApplication.processEvents()
			goal = self._get_current_goal()
			travelled = abs(goal.pose.position.x - oldX)
			# print("{0}. travelled = {1}. Button is pressed, no nav sent. dist = {2} ".format(i, travelled, dist))
			if (travelled >= (dist - tolerance) ):
				#Reset our variables tracking our distance travelled.
				oldX = goal.pose.position.x
				if (self.isForward):
					goal.pose.position.x += dist
				else:
					goal.pose.position.x -= dist
				self._send_nav_goal(goal)
				print("{0}. sending nav goal: {1} ahead. travelled = {2} ".format(i,goal.pose.position.x, travelled))
				travelled = 0
			i += 1

		if self.isForward:
			self.faceForward()
		else:
			self.faceBackward()

#PRIVATE FUNCTIONS
#^^^^^^^^^^^^^^^^
	def _send_twist(self, x_linear):
		if self.pub is None:
			return
		twist = Twist()
		twist.linear.x = x_linear
		twist.linear.y = 0
		twist.linear.z = 0
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 0

		# Only send the zero command once so other devices can take control
		if x_linear == 0:
			if not self.zero_cmd_sent:
				self.zero_cmd_sent = True
				self.pub.publish(twist)
		else:
			self.zero_cmd_sent = False
			self.pub.publish(twist)
	# Sends a nav goal to the bot. This is like sending it a position in space to go
	def _send_nav_goal(self, pose):
		self.nav_pub.publish(pose)
	# Returns a nav goal set to the current position of the robot with orientation of /initialpose to keep it along ze track.
	def _get_current_goal(self):
		(trans, rot) = self.listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))

		goal = PoseStamped()
		goal.header.frame_id = "/map"
		goal.header.stamp = rospy.Time.now()

		goal.pose.position.x = trans[0]
		goal.pose.position.y = trans[1]
		goal.pose.position.z = trans[2]
		if (self.isForward):
			goal.pose.orientation.z = self.start.pose.orientation.z
			goal.pose.orientation.w = self.start.pose.orientation.w
		else:
			quaternion = (self.start.pose.orientation.x,self.start.pose.orientation.y,self.start.pose.orientation.z,self.start.pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quaternion)
			roll = euler[0]
			pitch = euler[1]
			yaw = euler[2]
			yaw = yaw + pi

			newQuat = tf.transformations.quaternion_from_euler(roll,pitch, yaw)
			goal.pose.orientation.x = newQuat[0]
			goal.pose.orientation.y = newQuat[1]
			goal.pose.orientation.z = newQuat[2]
			goal.pose.orientation.w = newQuat[3]
		return goal

class PicButton(QAbstractButton):
    def __init__(self, pixmap, parent=None):
        super(PicButton, self).__init__(parent)
        self.pixmap = pixmap
        if self.pixmap.width() > 50:
        	self.pixmap = self.pixmap.scaled(50, 50, Qt.KeepAspectRatio, Qt.SmoothTransformation)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.drawPixmap(event.rect(), self.pixmap)


    def sizeHint(self):
        return self.pixmap.size()

## Start the Application
## ^^^^^^^^^^^^^^^^^^^^^
if __name__ == '__main__':
	app = QApplication( sys.argv )
	rospy.init_node('move')

	myviz = MyViz()
	myviz.resize( 1000, 500 )
	myviz.show()

	app.exec_()