#!/usr/bin/env python

## BEGIN_TUTORIAL
##
## Imports
## ^^^^^^^
##
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

## The MyViz class is the main container widget.
class MyViz( QWidget ):

	## MyViz Constructor
	def __init__(self):
#A comment for Alex
	#The visualizer
		QWidget.__init__(self)

		self.frame = rviz.VisualizationFrame()
		self.frame.setSplashPath( "" )
		self.frame.initialize()

		## The reader reads config file data into the config object.
		## VisualizationFrame reads its data from the config object.
		reader = rviz.YamlConfigReader()
		config = rviz.Config()
		reader.readFile( config, "map_and_img.rviz" )
		self.frame.load( config )

		self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )

	#The twist commands
		self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist) 
		self.zero_cmd_sent = False

	#For sending nav goals.
		self.nav_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)
		self.listener = tf.TransformListener()

	#Subscribe to initialpose.
		rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initialpose_callback)

	

#Disable unneeded views
		self.frame.setMenuBar( None )
		self.frame.setStatusBar( None )
		self.frame.setHideButtonVisibility( False )

		self.manager = self.frame.getManager()

		self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
		
		## Here we create the layout and other widgets in the usual Qt way.
		layout = QVBoxLayout()
		layout.addWidget( self.frame )
		
	   # speed_slider = QSlider( Qt.Horizontal )
		#speed_slider.setTracking( True )
	   # speed_slider.setMinimum( 0.0 )
	   # speed_slider.setMaximum( 1.0)
		#speed_slider.valueChanged.connect( self.onSpeedSliderChanged )
		#layout.addWidget( speed_slider )
		
		h_layout = QHBoxLayout()
		
		stop_button = QPushButton( "STOP" )
		stop_button.clicked.connect( self.onStopButtonClick )
		h_layout.addWidget( stop_button )

		debug_button = QPushButton( "Reset Position [NAV DEBUG]" )
		debug_button.clicked.connect( self.onDebugButtonClick )
		h_layout.addWidget( debug_button )
		
		self.fwd_button = QPushButton( "Move Forward[TWIST]" )
		self.fwd_button.pressed.connect( self.onFwdPress)
		# self.fwd_button.released.connect(self.onFwdRelease )
		h_layout.addWidget( self.fwd_button )

		turn_button = QPushButton( "Turn Around" )
		turn_button.clicked.connect( self.onTurnButtonClick )
		h_layout.addWidget( turn_button )
		
		layout.addLayout( h_layout )
		
		self.setLayout( layout )

	def initialpose_callback(self, initialpose):
	#Initialize our goal as on the start frame.
		self.start = PoseStamped()
		self.start.header.frame_id = "/map"
		self.start.header.stamp = rospy.Time.now()

		self.start.pose.position = copy.copy(initialpose.pose.position)
		self.start.pose.orientation = copy.copy(initialpose.pose.orientation)

	## Handle GUI events
	## ^^^^^^^^^^^^^^^^^
	##
	## After the constructor, for this example the class just needs to
	## respond to GUI events. Here is the slider callback.
	## rviz.Display is a subclass of rviz.Property. Each Property can
	## have sub-properties, forming a tree. To change a Property of a
	## Display, use the subProp() function to walk down the tree to
	## find the child you need.
   
	## switchToView() works by looping over the views saved in the
	## ViewManager and looking for one with a matching name.
	##
	## view_man.setCurrentFrom() takes the saved view
	## instance and copies it to set the current view
	## controller.
	def switchToView( self, view_name ):
		view_man = self.manager.getViewManager()
		for i in range( view_man.getNumViews() ):
			if view_man.getViewAt( i ).getName() == view_name:
				view_man.setCurrentFrom( view_man.getViewAt( i ))
				return
		print( "Did not find view named %s." % view_name )
#BUTTON CALLBACKS -------------------------------------------
	def onFwdPress(self):
		# while self.fwd_button.isDown():
		# self._send_twist(0.3)
		self.moveWhilePressed(0.3)
	# def onFwdRelease(self):
		# self.fwd_button.#Something about setting it to false now


	def onDebugButtonClick(self):
	#Tells robot to return to home base. Alex, you can continue editing here. 
		goal = PoseStamped()
		goal.header.frame_id = "/start"
		goal.header.stamp = rospy.Time.now()

		goal.pose.position.x = 0.0
		goal.pose.position.y = 0.0
		goal.pose.position.z = 0.0
		goal.pose.orientation.w = 1.0
		self._send_nav_goal(goal)

	def onStopButtonClick(self):
		self._send_twist(0.0)

	def onTurnButtonClick(self):
		# self.turnAround()
		self.turnAround()

	def turnAround(self):
		command = Twist()
		command.angular.z = 0.5
		now = rospy.Time.now()
		r = rospy.Rate(50) 
		while rospy.Time.now() - now < rospy.Duration(2*pi):
			QApplication.processEvents()
			self.pub.publish(command)
			r.sleep()
			
	#Turn around through nav goal.
	def navTurnAround(self):
		now = rospy.Time.now()
		# self.listener.waitForTransform("/odom", "/base_link", now, rospy.Duration(1.0))
		# my_pos = self.listener.lookupTransform("/odom", "/base_link", rospy.Time(0))
		# start_pos = self.listener.lookupTransform("/map", "/start", rospy.Time(0))
		self.listener.transformPose('', newMarker)

		goal = PoseStamped()
		goal.header.frame_id = "/map"
		goal.header.stamp = rospy.Time.now()

		goal.pose.position.x = my_pos[0][0]
		goal.pose.position.y = my_pos[0][1]
		goal.pose.position.z = my_pos[0][2]

		goal.pose.orientation.z = 1.0
		goal.pose.orientation.w = 0.0
		self._send_nav_goal(goal)

	#Function to be called as long as the move forward button is pressed. 
	def moveWhilePressed(self, velocity):
		now = rospy.get_time()
		#Speed up
		while rospy.get_time() - now < 2:
			QApplication.processEvents()
			x = rospy.get_time() - now            
			xVel = tanh(x) * velocity
			self._send_twist(xVel)
			if not self.fwd_button.isDown():
				break
		#Go while pressed
		while self.fwd_button.isDown():
			QApplication.processEvents()
			xVel = velocity
			self._send_twist(xVel)
		#Slow down
		now = rospy.get_time()
		while rospy.get_time() - now < 2:			
			QApplication.processEvents()
			print str( self.fwd_button.isDown() )
			x = 2 - (rospy.get_time() - now) 
			xVel = tanh(x) * velocity
			self._send_twist(xVel)

	# Give the turtlebot a distance to travel and a velocity and it follows the command + slows down accordingly as it reaches its destination.
	def moveAhead(self, distance, velocity):
		# By what facter we scale/lengthen the tanh function.
		s = 1
		# Integrate the x function below and set it equal to distance to find movingTime.
		movingTime = 1.0/s * acosh( exp( s/velocity * distance ) )
		now = rospy.get_time()
		while rospy.get_time() - now < movingTime:
			QApplication.processEvents()
			x = movingTime - (rospy.get_time() - now)            
			xVel = tanh(s * x) * velocity
			self._send_twist(xVel)

		# realign orientation	
		# now = rospy.Time.now()
		# self.listener.waitForTransform("/start", "/base_footprint", now, rospy.Duration(1.0))
		# my_pos = self.listener.lookupTransform("/start", "/base_footprint", rospy.Time(0))
		# # start_pos = self.listener.lookupTransform("/map", "/start", rospy.Time(0))
		# goal = PoseStamped()
		# goal.header.frame_id = "/start"
		# goal.header.stamp = rospy.Time.now()

		# goal.pose.position.x = my_pos[0][0]
		# goal.pose.position.y = my_pos[0][1]
		# goal.pose.position.z = my_pos[0][2]

		# goal.pose.orientation.w = 1.0
		# self._send_nav_goal(goal)


#PRIVATE COMMANDS ---------------------------------------------
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


## Start the Application
## ^^^^^^^^^^^^^^^^^^^^^
##
## That is just about it. All that's left is the standard Qt
## top-level application code: create a QApplication, instantiate our
## class, and start Qt's main event loop (app.exec_()).
if __name__ == '__main__':
	app = QApplication( sys.argv )
	rospy.init_node('move')

	myviz = MyViz()
	myviz.resize( 1000, 500 )
	myviz.show()

	app.exec_()