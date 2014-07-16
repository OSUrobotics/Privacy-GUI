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
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, PointStamped
from actionlib_msgs.msg import GoalID
# from pr2_controllers_msgs.msg import PointHeadActionGoal
from control_msgs.msg import PointHeadActionGoal

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
		#Now you can grab this filepath from either roslaunch remote_nav myviz and using the launch file or just rosrun.
		config_file = rospy.get_param('remote_nav/rviz_config', package_path + "/rviz/pr2_map_img.rviz")
		reader.readFile( config, config_file )
		self.frame.load( config )

		self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )
		self.setWindowIcon(QIcon(package_path +'/images/icon.png'))


	#The twist commands
		# self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist) 
		# self.zero_cmd_sent = False

	#For sending nav goals.
		nav_topic = rospy.get_param("remote_nav/nav_topic", "/move_base_simple/goal")
		self.nav_pub = rospy.Publisher(nav_topic, PoseStamped)
	#A publisher to literally tell dis bisnatch to cancel all goals.
		cancel_topic = rospy.get_param("remote_nav/cancel_topic", '/move_base/cancel')
		self.cancel_pub = rospy.Publisher(cancel_topic, GoalID)

	#We choose in our implementation to move the head using the preexisting head trajectory controller.
		head_topic = rospy.get_param("remote_nav/head_topic", 'are you a turtlebot? This no for turtlebot')
		self.head_pub = rospy.Publisher(head_topic, PointHeadActionGoal)
	#We need be transformin these mofuckin frames.
		self.listener = tf.TransformListener()

	#Is the robot facing forward along our track?
		self.isForward = True
	#Get the track_length for our 1D start track.
		self.track_length = rospy.get_param('remote_nav/track_length', 5.0)
		self.robot_frame = rospy.get_param('remote_nav/robot_frame', "/base_footprint")
	

	#Disable unneeded views and more visualization setup
		self.frame.setMenuBar( None )
		self.frame.setStatusBar( None )
		self.frame.setHideButtonVisibility( False )
		self.manager = self.frame.getManager()
		self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
		
	##LAYOUT
	##^^^^^^
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
		layout.addWidget( self.stop_button, 6, 1 )

		# debug_button = QPushButton( "Reset Position" )
		# debug_button.clicked.connect( self.onDebugButtonClick )
		# debug_button.setToolTip('Set a navigation goal at the starting point')
		# h_layout.addWidget( debug_button )

		# reset_dir_btn = QPushButton( "Reset Orientation" )
		# reset_dir_btn.clicked.connect( self.onResetDirButtonClick )
		# reset_dir_btn.setToolTip('Reset to the original orientation')
		# h_layout.addWidget( reset_dir_btn )
		

		self.fwd_button = PicButton(QPixmap(package_path + "/images/up.png"))
		self.fwd_button.setClickPix(QPixmap(package_path + "/images/upDark.png"))
		# self.fwd_button = QPushButton("Move Forward")
		self.fwd_button.pressed.connect( self.onFwdPress )
		self.fwd_button.setToolTip('While held, the robot will move forward')
		layout.addWidget( self.fwd_button, 4, 1 )
		layout.setAlignment(self.fwd_button, Qt.AlignHCenter)

		turn_button = PicButton(QPixmap(package_path + "/images/rotate.png"))
		turn_button.setClickPix(QPixmap(package_path + "/images/rotateDark.png"))
		# turn_button = QPushButton( "Turn Around[ALEX DEBUG - Nav Goals]" )
		turn_button.clicked.connect( self.onTurnButtonClick )
		turn_button.setToolTip('The robot will turn around 180 degrees')
		# layout.addWidget( turn_button, 5, 1 )
		# layout.setAlignment(turn_button, Qt.AlignHCenter)

		# turn_twist_button = QPushButton( "Turn Around[ALEX DEBUG - Twist message]" )
		# # turn_button.clicked.connect( self.onTurnButtonClick )
		# turn_twist_button.clicked.connect( self.onTurnTwistButtonClick)
		# turn_twist_button.setToolTip('The robot will turn around 180 degrees')
		# h_layout.addWidget( turn_twist_button )



		look_left_btn = PicButton(QPixmap(package_path + "/images/left.png"))
		look_left_btn.setClickPix(QPixmap(package_path + "/images/leftDark.png"))
		look_left_btn.pressed.connect( self.onLeftButtonClick )
		# layout.addWidget(look_left_btn, 2, 0)
		# layout.setAlignment(look_left_btn, Qt.AlignLeft)

		look_right_btn = PicButton(QPixmap(package_path + "/images/right.png"))
		look_right_btn.setClickPix(QPixmap(package_path + "/images/rightDark.png"))
		look_right_btn.pressed.connect( self.onRightButtonClick )
		# layout.addWidget(look_right_btn, 2, 2)
		# layout.setAlignment(look_right_btn, Qt.AlignRight)
		

		#Finalizing layout and placing components
		h_layout.addWidget(look_left_btn)
		h_layout.setAlignment(look_left_btn, Qt.AlignRight)
		h_layout.addWidget(turn_button)
		h_layout.setAlignment(turn_button, Qt.AlignHCenter)
		h_layout.addWidget(look_right_btn)
		h_layout.setAlignment(look_right_btn, Qt.AlignLeft)

		layout.addLayout( h_layout, 5, 1 )	
		self.setLayout( layout )


## Handle GUI events
## ^^^^^^^^^^^^^^^^^

	def closeEvent(self, event):

		reply = QMessageBox.question(self, 'Message',
		"Are you sure to quit?", QMessageBox.Yes | 
		QMessageBox.No, QMessageBox.No)

		if reply == QMessageBox.Yes:
			event.accept()
		else:
			event.ignore()

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
		self.moveNav()


	def onDebugButtonClick(self):
	#Tells robot to return to home base.
		goal = self._get_start_pose()

		self._send_nav_goal(goal)
		self.isForward = True

	def onStopButtonClick(self):
		QApplication.processEvents()
		self._cancel_goals()

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
	def onLeftButtonClick(self):
		self.lookLeft()
	def onRightButtonClick(self):
		self.lookRight()

## MOVE ZE HEAD FUNCTIONS
## ^^^^^^^^^^^^^^^^^^^^

	def lookLeft(self):
		goal = PointHeadActionGoal()

		#the point to be looking at is expressed in the "base_link" frame
		point = PointStamped()
		point.header.frame_id = "base_link"
		point.header.stamp = rospy.Time.now()
		point.point.x = 1.0
		point.point.y = 0.5 
		point.point.z = 1.20
		goal.goal.target = point

		#we want the X axis of the camera frame to be pointing at the target
		goal.header.frame_id = "base_link"
		goal.goal.pointing_frame = "high_def_frame"
		goal.goal.pointing_axis.x = 1
		goal.goal.pointing_axis.y = 0
		goal.goal.pointing_axis.z = 0
		goal.goal.max_velocity = 1.0
		self. head_pub.publish(goal)
	def lookRight(self):
		pass

## NAVIGATION FUNCTIONS
## ^^^^^^^^^^^^^^^^^^^^
	#Face forward along our track.
	def faceForward(self):
		self.isForward = True
		goal = self._get_pose_from_start()
		goal.pose.position.y = 0
		goal.pose.orientation.z = 0.0
		goal.pose.orientation.w = 1.0

		self._send_nav_goal(goal)
		print ("Now facing forward.")

	#Face 180 degrees from the forward position.
	def faceBackward(self):
		self.isForward = False
		#Grab where we currently are.
		goal = self._get_pose_from_start()
		#realign and turn around.
		goal.pose.position.y = 0
		goal.pose.orientation.z = 1.0
		goal.pose.orientation.w = 0.0
		self._send_nav_goal(goal)
		print ("Now facing backward.")
	def moveNav(self):
		if (self.isForward):
			goal = self._get_end_pose()
			self._send_nav_goal(goal)
		else:
			goal = PoseStamped()
			goal.header.frame_id = "/start"
			goal.pose.orientation.z = 1.0
			goal.pose.orientation.w = 0.0

			self._send_nav_goal(goal)

		while self.fwd_button.isDown():
			QApplication.processEvents()
		self._cancel_goals()
		
	#Moves ahead via nav goals while the button is pressed.
	# def moveNav(self, dist):
	# 	toStart = self._get_pose_from_start()

	# 	# Keep track of how far we've travelled in order to only send new nav goals when need be. 
	# 	travelled = 0.0
	# 	goal = self._get_pose_from_start()
	# 	#oldX indicates our first nav x position.
	# 	oldX = goal.pose.position.x
		
	# 	#If we say "travel 1 meter" the robot will probably just travel ~0.9 meters. This trys to account for that by grabbing the x, y tolerance.
	# 	tolerance = rospy.get_param("/move_base/TrajectoryPlannerROS/xy_goal_tolerance", "0.25")
		
	# 	i = 0
	# 	#give an initial command to go.
	# 	goal.pose.position.y = 0.0
	# 	goal.pose.orientation.z = 0.0
	# 	goal.pose.orientation.w = 0.0
	# 	if (self.isForward):
	# 		goal.pose.position.x += dist
	# 		goal.pose.orientation.w = 1.0
	# 	else:
	# 		goal.pose.position.x -= dist
	# 		goal.pose.orientation.z = 1.0

	# 	self._send_nav_goal(goal)

	# 	while self.fwd_button.isDown():
	# 		QApplication.processEvents()
	# 		goal = self._get_pose_from_start()
	# 		travelled = abs(goal.pose.position.x - oldX)
	# 		if (travelled >= (dist - tolerance) ):
	# 			#Reset our variables tracking our distance travelled.
	# 			oldX = goal.pose.position.x
	# 			travelled = 0

	# 			goal.pose.position.y = 0.0
	# 			goal.pose.orientation.z = 0.0
	# 			goal.pose.orientation.w = 0.0
	# 			if (self.isForward):
	# 				goal.pose.position.x += dist
	# 				goal.pose.orientation.w = 1.0
	# 			else:
	# 				goal.pose.position.x -= dist
	# 				goal.pose.orientation.z = 1.0
	# 			self._send_nav_goal(goal)
	# 		i += 1
		#TODO: THIS RESULTS IN 
		#THE ROBOT GOING  BACKWARDS DUE TO SENDING MSG TO CURRENT POSITION BEFORE FULLY STOPPING
		#PLEASE FIX THIS
		# if self.isForward:
		# 	self.faceForward()
		# else:
		# 	self.faceBackward()

	#Rotate the robot exactly 180 degrees with a twist command
	# def turnAround(self):
	# 	command = Twist()
	# 	command.angular.z = 0.5
	# 	now = rospy.Time.now()
	# 	r = rospy.Rate(50) 
	# 	while rospy.Time.now() - now <= rospy.Duration(2*pi):
	# 		QApplication.processEvents()
	# 		print (rospy.Time.now() - now)
	# 	#An attempt to stop it while it is in motion
	# 		if self.stop_button.isDown():
	# 			command.angular.z = 0.0
	# 			break
	# 		self.pub.publish(command)
	# 		r.sleep()
	# 	command.angular.z = 0.0
	# 	self.pub.publish(command)
			

		

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
	#Just returns a goal in the /start frame.
	def _get_start_pose(self):
		goal = PoseStamped()
		goal.header.frame_id = "/start"

		goal.pose.orientation.w = 1
		return goal
	#Returns the pose of the END of the track defined by start.
	def _get_end_pose(self):
		pose = PoseStamped()
		pose.header.frame_id = "/start"
		pose.pose.position.x += self.track_length
		pose.pose.orientation.w = 1.0
		return pose

	def _cancel_goals(self):
		goalID = GoalID()
		self.cancel_pub.publish(goalID)

	# def _send_twist(self, x_linear):
	# 	if self.pub is None:
	# 		return
	# 	twist = Twist()
	# 	twist.linear.x = x_linear
	# 	twist.linear.y = 0
	# 	twist.linear.z = 0
	# 	twist.angular.x = 0
	# 	twist.angular.y = 0
	# 	twist.angular.z = 0

	# 	# Only send the zero command once so other devices can take control
	# 	if x_linear == 0:
	# 		if not self.zero_cmd_sent:
	# 			self.zero_cmd_sent = True
	# 			self.pub.publish(twist)
	# 	else:
	# 		self.zero_cmd_sent = False
	# 		self.pub.publish(twist)

	# Returns a nav goal set to the current position of the robot with orientation of /initialpose to keep it along ze track.
	# def _get_current_pose(self):
	# 	(trans, rot) = self.listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))

	# 	goal = PoseStamped()
	# 	goal.header.frame_id = "/map"
	# 	goal.header.stamp = rospy.Time.now()

	# 	goal.pose.position.x = trans[0]
	# 	goal.pose.position.y = trans[1]
	# 	goal.pose.position.z = trans[2]
	# 	if (self.isForward):
	# 		goal.pose.orientation.z = self.start.pose.orientation.z
	# 		goal.pose.orientation.w = self.start.pose.orientation.w
	# 	else:
	# 		quaternion = (self.start.pose.orientation.x,self.start.pose.orientation.y,self.start.pose.orientation.z,self.start.pose.orientation.w)
	# 		euler = tf.transformations.euler_from_quaternion(quaternion)
	# 		yaw = euler[2] # yaw gonna make me lose my mind,
	# 		pitch = euler[1] #up in pitch
	# 		roll = euler[0] #up in roll (see photo at bottom)
	# 		yaw = yaw + pi

	# 		newQuat = tf.transformations.quaternion_from_euler(roll,pitch, yaw)
	# 		goal.pose.orientation.x = newQuat[0]
	# 		goal.pose.orientation.y = newQuat[1]
	# 		goal.pose.orientation.z = newQuat[2]
	# 		goal.pose.orientation.w = newQuat[3]
	# 	return goal




#SPECIAL SNOWFLAKE CLASSES
#^^^^^^^^^^^^^^^^
class PicButton(QAbstractButton):
	def __init__(self, pixmap, parent=None):
		super(PicButton, self).__init__(parent)
		self.pixmap = pixmap
		self.clickpix = pixmap
		if self.pixmap.width() > 50:
			self.pixmap = self.pixmap.scaled(50, 50, Qt.KeepAspectRatio, Qt.SmoothTransformation)

	def setClickPix(self, pixmap2):
		self.clickpix = pixmap2

	def paintEvent(self, event):
		painter = QPainter(self)
		if self.isDown():
			painter.drawPixmap(event.rect(), self.clickpix)
		else:
			painter.drawPixmap(event.rect(), self.pixmap)

	def sizeHint(self):
		return self.pixmap.size()

	# def mousePressEvent (self, event):
	# 	self.setPixmap(self.clickpix)

	# def mouseReleaseEvent (self, event):
	# 	self.setPixmap(self.pixmap)


## Start the Application
## ^^^^^^^^^^^^^^^^^^^^^
if __name__ == '__main__':
	app = QApplication( sys.argv )
	rospy.init_node('remote_nav')

	myviz = MyViz()
	myviz.resize( 1000, 500 )
	myviz.show()

	app.exec_()

"""
                                                                                             `   `    `,:::,:,:::::,,:,,,,,,.`      `                                                                          
                                                                                           ``     ``,::,::,,::::::::,,,,,,,,,,..``                                                                             
                                                                                            `   `.,:::::::::::::::::,,,,,,,,,,,,,,`   ``                                                                       
                                                                                            ```.::::::::::::::::::::,,,,,,,,,,,,,,,,``                                                                         
                                                                                           ` `,::::::::::;::;:::::::,,,,,,,,,,,,,,,,,.`                                                                        
                                                                                            .:;::::;;;;;;;;;::::::::,,,,,,,,,,,,,,,,,,,`                                                                       
                                                                                      ```  .;;;;;;;;;;;;;:;;::::::::::,,,,,,,,,,,,,,,,,,.  `                                                                   
                                                                                      `   .;::;;;;;;;;;;;:;;;;::::::::::::,,,,,,,,,,,,,,,.                                                                     
                                                                                       ` ,::::;;;;;;;;;;;:;::::::::::::::::,,,,,,,,,,,,,,,. `                                                                  
                                                                                        .::;;;;;;;;;;;;;;;;;;;;;;;;;;:::::::,,,,,,,,,,,,,,,.                                                                   
                                                                                       .;:;;;;;;;;;;;;;;;;;;;;;;;;;;;::::::::,,,,,,,,,,,,,,,`  `                                                               
                                                                                   `` .:;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;:::::::,,,,,,,,,,,,,,,`  ``                                                             
                                                                                 `   `:;;;;;;'''';;;;;;;;;;;;;;';;;;;;;;;::::,:,,,,,,,,,,,,,,,``                                                               
                                                                                `    ,;;;;'''''';;;;;;'';;;;;;;''';;;;;;;::::,,,,:,,,,,,,,,,,,.  `                                                             
                                                                                    .;;;;;'''''';;;;;;'''''''''''';;;;;;;:::::,,:,,,,,,,,,,,,,,.                                                               
                                                                                 `  :;;;;''''''';;''''''''''''''''';;;;;;;::,,,,,,,,,,,,,,,,,,,,`                                                              
                                                                                 ` `;;;'''''''''''''''''''''''''''';;;;;:;:::,,,,,,,,,,,,,,,,,,,.` `                                                           
                                                                                  `:;;;''''''''''''''''''''''''''''';;;;;;:::,,,,,,,,,,,,,,,,,,,,.                                                             
                                                                                  `;;;;''''''''''''''''''''''''''''';;;;;;:::,,,,,,::,,,,,,,,,,,,,`    `                                                       
                                                                                  .;;;''''+'''''''+'''''++''''''''''';';;;::,,,,,,,,:,,,,,,,,,,,,,. `                                                          
                                                                                ` :;';'''''''''''+++++''+'''''''''''';;;;;;:,,,,,,,,,,::,,,,,,,,,,,` `                                                         
                                                                               ` `:;;''''''''''''++'++++''+++''''''''';;;;;::,,,,,,,,,,:,,,,,,,,,,,.                                                           
                                                                             `   .;;;''''''''''''+''+++++''++''''''''';;;;;:::,,,,,,,:,,,,,,:,,,,,,,  `                                                        
                                                                                 ,;;;'''''''''''++'+++++++'++''''''''';;;;;:::,,,,,,,,,,,,,,:,,,,,,,`                                                          
                                                                                `;;;;'''''''''''++'++++++++++++'''''''';;;::::,,,,,,,,,,,,,,:::,,,,,.  `                                                       
                                                                              ` .;;;;'''''''''''''+++++++++'++++'''''''';;::::,,,,,,,,,,,,,,::::,,,,,                                                          
                                                                                ,;;;;''''''''''''''+++++++++++++'''''''';;::::,,,.,,,,,,,,:,::::,,,,,`                                                         
                                                                               `,;;;;'''''''''''''++++++++++++++''''''';;;::::,,,,,,,,,,,:::::::,,,,,`                                                         
                                                                               `:;;;;''''''''''''''+++++++++++++'''''''';;:::::,,,,,,,,,,:::::::,,,,,.                                                         
                                                                            `  .:;;;;''''''''''''''+++++++++++++'''''''';;:::::,,,,,,,,:,:::::::,,,,,. `                                                       
                                                                               .;;;;;''''''''''''''++++++++++++++''''';;;;:::::::,,,,,,:::::::::,,,,,, `                                                       
                                                                               ,;';;;'''''''''''''''+++++++++++++'''';;;;;;:::::::,,,:::::::;;::,,,,,,                                                         
                                                                               :;';;;'''''''''''''''+++++++++++++'''';;;;;;;::,::,:,,:,,::::;;::,:::,.                                                         
                                                                              `:;;;;;';''''''''''''''++++++++++++'''';;';;;;:::::::,::::::;;;;::::,,,.           `                                             
                                                                      ` `     `:;;;;;;;'''''''''''''''+++++++++++''''';'';;;:::::::::::::;:;;;;::::,,.                                                         
                                                                              .:;;;;;;'''''''''''''''''+++++++++++'''';;';;;:::::::::::::;::;;;:::::,.                                                         
                                                                         `    .;;;;;;;'''''''''''''''''+++++++++++''''''';;;;::::::::::;;;;;;;;:;;::,.                                                         
                                                                      `       ,;;;;;;;'''''''''''''''''+++++++++++''';;';;;;;;::::::;::;;;;;;;;;:;:::.                                                         
                                                                       `:,`   :';';;:;'''''''''''''''''''+++++++++''''''';;;;:;::;::::;;;;;;;;;;;;:::.                                                         
                                                                     ``;+#'  `;'';;;::''''''''''+++'''''''++++++++''''''';;;;;:;;;;;;;;;;;;;;;:;;::::.                                                         
                                                                      .;'#@:``;''';;:;''''''''''+++++''''''+++++++''''';';;;;;;;;;;;;;;;;;;;;:;;;::::`                                                         
                                                                      '#'+##``'''';;;;''''''''''+++++'''''+++++++++'''''';;;;;;;;;;;;;;;;;;;;;;:;::,,`                                                         
                                                                     .###'##@;'''';:;''''''''''''++++++'''+++++++++''';';;;;;;;;;;;;;;;;;;;;;;;;;::,,`                                                         
                                                                     :##+'####+';';;;'''''''''''+++++++++++++++++++'''';;;;;;;;;;;;;;;;;;;;;;;;;:::,,  `                                                       
                                                                     ;##''+###+';;;;''''''''''''++++++++++++++++++++''';;;;;;;;;;'';;;;;;;;;;;;;::,,.                                                          
                                                                     ;@+'''+#++';;;;''''''''''++'+++++++++++++++++++''';;;;;;;;;'''';;;;;;;;;;;;,,,,`                                                          
                                                                     ;#'''''+++';;;;'''''''''''''+++++++++++++++++++'';;;;;;;;;;;;;;;;;;;;;;;;;:,:,,                                                           
                                                                     :+'++'''''';;;''''''''++++'++'++++++++++++++++''';;;;;;;;;;;;;;;;;;;;;;;;;,,,:, `                                                         
                                                                     :+++++''''+;;;'''''+++++++++'++++++++++++++++''''';;;;;;:::;;::;;;;;;;;;;:,:,:.   `                                                       
                                                                     :+++##+'''+;;;''''+++++++++++''++++++++++++++'''''';;;;;;::::::;;;;;;;;;;,,::,`                                                           
                                                                     ;####+++'+;;;''''''''++#+++++++++++++'''++++''''''';;;;:::::::::;;;;;;;;:,,,,,` `                                                         
                                                                    `'####+'+'';;;''';;''''''++##++++++''+'''''''''''''';;;;:::::::::;;;;;;;;:,:::. `                                                          
                                                                    `'##@@+'++';;''''';;'''''''+###+++++'''''''''''''''';;;::::::::::;;;;;;';:,,:,`                                                            
                                                                `   `'#@@@+''';;;'''''''''+++''''++##+++''''''''''''''';;';;:::::::::;;;;;;;:,,::. ``                                                          
                                                                    `'+#@@+'';;'''''''''+########++++#+++''''''''''''';;';;;::::::::::;;;;;;:,:::`                                                             
                                                                     ;''+@''';''''''''''+++++###@@@##+++'''''''''''''''''';;;:::::::,:::;;;;:,::,                                                              
                                                                     :#'+#'';'''''''++'''#@##+@@@##@@#++++''''''';;;;;;;'';;;:::::,,::::;;;;:,::` `    `                                                       
                                                                     .#++@#';''''''++++'''+#,.'#@@@##@#++''''''''';;;;'''';;;::;::::::::;;;;:::,  ``` `                                                        
                                                                     `+++@#;'''''''++++++''#'`;++;+###@#+'''''''''';;'''''''''''';;;;;:::;;;:::` .;,,`                                                         
                                                                      ;+'++;'''''''++++#''''#;,'+++:+###++''''''';;;''''''+++++++'''';;;::;;::;:;;,,:. `                                                       
                                                                      .+''+'''''''''+++#+'';'##+++'+#####+''''''';;;'''+++##+++#++++'''';;;;:::,:;;::`                                                         
                                                                     ` ;''+;''''''''+++++''';''#@@@@@@###+''''''';;''''+++####++'+''''''';;;::::;''#'  `                                                       
                                                                       `;'+;'''''''''++++'''''''++++#####+''''''';;'''+'+@@@@@###+''''''';;;,;;;''#@:                                                          
                                                                        `'+;''+'''''''++++''''++++++####++''''''''''+++#@@#####+++##+''';;;;:';'''##.                                                          
                                                                         .';'''''';''''+++++'''+++++##+#+++''++'''''++####'@@@@@##++#++';;;::';'';++  `                                                        
                                                                        ` .;'''''''''''++++++++++++++#+++++''+++';'+++#+##;##+;'+'@#+#++';;::'''';+; `                                                         
                                                                        ` ,'''''''';''''+++++++++++++++++++++'+';:''+++##@#++'':.,+@#+++'';::'+++'+.                                                           
                                                                          ,';'''''';;''''++++++++++++++++++'''+';,;'++++####@+;..'+##++''';::#+++';                                                            
                                                                          ,';''''''';'''''+++++++++'''+++++''++';,:;++++##++++';''''''''';;::#+++'. `                                                          
                                                                       `  :'''''''';''''''++++++++''''''+++''+'':,:;'+++++++''''''';;;;;;;::,++++;                                                             
                                                                       `  :'''''''';''''''+'+++++'''''''''''++';:,::;'++++++'''''';;::::;;:::#++',  `                                                          
                                                                          ;'''''''''''''''''''+++'''''''++''++';::::;;''++'+''''';;:,,,::::::+;;'`                                                             
                                                                         `;'''''''''''''''''''+++'''''''++++++';::::;;;;''''''';;::,,,,,;;::::,;:                                                              
                                                                         `;'''''''''''';'''''''+++''''''++++++':,::::;;;;;''''';::,,,,,::::,,:':``                                                             
                                                                         `;'''''''''''';'''''''+++'''''''+++'';,,,,::;;;;;;;';;;::,,,,:::;::,''`                                                               
                                                                         .'''''''''''''''''''''+++''++''++++';:,.,,,:;;;:;:;;;;;:::,:::;;;::,:. `                                                              
                                                                     `   .''''''''''';''''''''''++'++++''+++';:..,,,::;;:::::;;;:;::::;;;;:::,`                                                                
                                                                         ,+'''''''''''''''''''''+'''+++''+++';:,..,,:;;;;;;:;;;;;;;;;;;;;;:;:.                                                                 
                                                                         ,+'''''''''''''''''''''+'''+++''++++;:,,,,,,;'';;;;:;;;';;;;;;;;;::,`                                                                 
                                                                       ` ,+'''''''''''''''''''+++''''''''++++':,,:,,,;+';;;;;;;;;;;;'''';:::,                                                                  
                                                                         ,'''''''''''''''''''+++++''''''''+++';:::::,;+''';;;;;;;;;''''';::`. `                                                                
                                                                         ,+''''''''''''''''''+++++'''''';''''';;;;;::;++'';;;;;;;''''''';;, .                                                                  
                                                                         .+''''''''''''''''''++++##+'+++''''';;;;;;;;;++''';;;;;'''''''';'`,,                                                                  
                                                                         .''''''''''''''''''+++++++#+''''''';;''+'''''++'''''';;'''''''':+,,                                                                   
                                                                         .+'''''''''''''''''++++++++++++#+''''++++++'''+''''''''''''''''``.`                                                                   
                                                                       ` `'''''''''''''''''++++++++++#####+'+++++++';''++'''''''''''''+:                                                                       
                                                                          ''''''''''''''''++++++++++++##+++++++++';;;''++'''''''''++++'`  `                                                                    
                                                                          ;+'''''''''''''''+++++++++++##+++++++++;;;;;''++''''''++++++;   `                                                                    
                                                                        ` .+''''''''''''''''++'++#++++#+#+++++++';;;;;'''''''''+++++++. `                                                                      
                                                                          `''''''''''''''''++############+###+'';;;;;''''''''++++++++'`                                                                        
                                                                           ,''''''+'''''''+++################++'';'''';''''++++++++++,                                                                         
                                                                           .'''''++''''''++++++############@#+++''';;';''''+++++++++'`    ``                                                                   
                                                                           `+''''''''''''+''''''+++++##+##@@##+#++'';'''''+++++++++++;:,.`                                                                     
                                                                 `   `      :+'''''''+''+'+++';''''''''+++++'''++#+'''''''++'++++++++##@##+':.      `                                                          
                                                                    `  `.:'##'''''''''''+'++@@#+''''''++'''''';;;'+++'''''++++++++++.###@@@@@@#+;,.``                                                          
                                                                 `   ,;+#@@@@#''''''+'''+'''+@####+''''''''''++';;'+#++'''+++++++++'.:@##@@@@@@@@@##+;,.`                                                      
                                                               `  `,#@@@@@@@@#+'''''+''+'''''#+'+####+''''''''''+';+++++'++++++++++;;.+@++#@@@@@@@@@@@@##+';,.```                                              
                                                                 ,+@@@@@@@@@@#++'''''+++''''''+''++#########+++++++++##+++++++++++;;#;,##++#@@@@@@@@@@@@@@@@@##+'::.`                                          
                                                               `'@#@@@@@@@@@@#'++''+'+++''''''+'''++#++++######++@##++++++++++++++':++,;##++#@@@@@@@@@@@@@@@@@@@@@##+',``                                      
                                                           `  ,#@@@@@@@@@@@@@#'++''+++++'''''''''''+++';++'':,:''++++#++++'++++++';::;::;#++++##@@@@@@@@@@@@@@@@@@@@@@#+':.`                                   
                                                             ;@#@@@@@@@@@@@@@;+++''+'+++''''''''''''''''+'';::;'';'++++++++++++#';;::#:,##+##+++###@@@@@@@@@@@@@@@@@@@@@@##+,`  `                              
                                                           .+@@#@@@@@@@@@@@@@''+##'+'+++'''''''''';''''''''';;;';'++++++++++++#+';;::+#.+@##+#++####@@@@@@@@@@@@@@@@@@@@@@@@##',`  `                           
                                                          ,#@@@@@@@@@@@@@@@';''++#++++++''''''''++';;;''''';;;;''++++++++++++++'';:;:+#;.###++++######@@@@@@@@@@@@@@@@@@@@@@@@##+,                             
                                                       ``:#@@@@@@@@@@@@@@@;:+#+++##+++++'''''''''++++''';;;;'''+++++++++++++#+'';;;:::+,:.##############@@@@@@@@@@@@@@@@@@@@@@@@#+:`                           
                                                      ` ;#@@@@@@@@@@@@@@@@+:##++++####+++''''''''++++###+++++++++++++++++++#+''';;;:;':#+.###############@@@@@@@@@@@@@@@@@@@@@@@##+;``                         
                                                       ,#@@@@@@@@@@@@@@@@@@'##++++######+++'''''''+++++++++#+++++++++++++##+'''';;::'#::+'#################@@@@@@@@@@@@@@@@@@@@@@#++:`     `                   
                                                      ,#@@@@@@@@@@@@@@@@@+;'##+++++####+++++''''''++++++++++++++++++++++##++'''';;;:+'+,:###############@###@@@@@@@@@@@@@@@@@@@@@#+++;`                        
                                                     `+@@@@@@@@@@@@@@@@@++,#@#+++++#######+++'++++++++++'+++++'+###++++#+++'''';;;::+';,,,##########@########@@@@@@@@@@@@@@@@@@@@##+++;`  `                    
                                                     :@@@@@@@@@@@@@@@@@@;+,@@##++++++######++++++++++++'''++++'++#++++#+++'''''';;::#:'++.################@#@#@@@@@@@@@@@@@@@@@@@##++++'.                      
                                                    `#@@@@@@@@@@@@@@@@@@@@;+@###+++++#########+++++++++++'''+++++++++#+++++''''';;::#+.#';@###@########@##@#@#@@@@@@@@@@@@@@@@@@@##++++++:`                    
                                                    ;@@@@@@@@@@@@@@@@@@@###;@###++++++##########++++++#++++'+'+#+'+##+++++'''''';;:;##,+'####@@@###@###@##@#@##@@@@@@@@@@@@@@@@@####++++++':                   
                                                `  `+@@@@@@@@@@@@@@@@@@#';@+@####++++++#############+###++++++++++##+++++++''''';;:;++;.,####@@@###@###@##@@@@@@@@@@@@@@@@@@@@@@####++++++++'`                 
                                                 `:'#@@@@@@@@@@@@@@@@@@';;##@@####++++++#########++##+##+####+#++##+++++++''''';;;:+++:',+##@@@@###@###@####@@@@@@@#@#@#@@@@@@@######+#+++++++.                
                                                ,+###@@@@@@@@@@@@@@@@@@@#;#@@@#####++++++####################+####++++++++'''''';;;#:###;+@##@@###@##@#@####@@@@@@######@@@@@@@######+++++++++;                
                                           `  `:#####@@@@@@@@@@@@@@@@@@@@+#@@@@#####++++++++#####################+++++++++'''''';;'@:,@#+###@@@##@###@######@@@@@#+#####@@@@@@########++#+++++'`               
                                          ` `:'######@@@@@@@@@@@@@@@@@@@#@+@@@@@#####++++++++++###@############++++++++++++''''';;#@#.#;+##@@@@#@@###@#######@@@@+#######@@#@@########+++++++#:``              
                                        `  ,;########@@@@@@@@@@@@@@@@@@#,###@@@@#####+++++++++++++++#########+++++++++++++++'''';'#@;,:.+##@@@###@###@#@#####@@@#'@#####@###@@###############;`..`             
                                    ` `` .''+########@@@@@@@@@@@@@@@@@@+,+++@@@@@#####+++++#+++++++++++++++++++++++++++++++++''''#@++:+;:##@@##@@@#@#@###@###@@@+###@@####@@@@@@@#########@+:`...`             
                                       `:'''#@#######@@@@@@@@@@@@@@@@@@@+;#@@@@@@@######++++###+++++++++++++++++++++++++++++''''+@@:'##'+#@@@##@@##@#@###@@##@@#+@##@###@#@#+;;;'+++++''';,`.,,...`            
                                      `'++'+@@#@######@@@@@@@@@@@@@@@@@@@+#@@@@@@@@######++++##+++++##++++++##+++++++++++++++''+@@@+,@#'##@@#@#@@##@#@###@@@#@@+#@#@###@@'++,`....```````...,,,....`           
                                     .'#'+'#@#@@######@@@@@@@@@@@@@@@@@@@@#@@@@@@@@@@#####+++#####+#####+###++####++++++++++++##@@@+.+'+@@###@#@###@#@####@@@@#+@#@@###@+'++:.,,:::::,,,,,::,.......`          
                                    ,++++++#@#@@@@####@@@@@@@@@@@@@@@@@@:@+'@@@@@@@@@@####+++++#+###########++#####+++++#+####@@#@#',,,;@###@@#@#@#@#@####@@@@+#@#@@#@#####+:,,,,,,,,,,,,,.....,.,...`         
                                   ,+'#++++#@@#@@#####@@@@@@@@@@@@@@@@@@;;##@@@@@@@@@@@@@####+++++++++##++++++#############@@@##@@#:,+;,@#@@#@#@#@@@#@###@@@@#+@#@@###@+;@#':,,.............,,,,.,.,.`         
                                 `,+'+@'++++@@@@@##@###@@@@@@@@@@@@@@@@@@'#@@@@@@@@@@@@@@@@@@####################@@@@@@@#@@@@###@@:###'+@##@#####@@@#@####@@@##@#@@@##@''@#':;;::,::,,,,::::,,,..:,,,.`        
                                .:+'+##'++++@@@@@#@@##@@@@@@@@@@@@@@@@@@@@#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@###@@@,:@@'###@#####@@@@#@#@@#@@@+@@@@@#@@##+@+;:';;:::::::::::,,,...:,.,,.        
                              `,;''+#@#+++++@@@@@@@#@@@@@@@@@@@@@@@@@@@@#@#'@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@####@@@@;.+;+#########@@@@#@#@@@@@##@#@@@@@@#;#@+::;::::::;;;::,,,,,.`.;,.,,.`       
                            `.;''+'+#@#++#++#@@@@#@@@@@@@@@@@@@@@@@@@@@@''@;#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@###@@@#@@@;`:,'#@@####@#@@@@@@#@@@@@+@#@@@@@@@+;@#+:;;:::::::::,,,,,,...,;:..,..       
                           .:;;'+'+###+++++++@@#@@###@@@@@##@@@@@@@@@@@@@:'#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@###@@@@@@@@#':':;@######@#@@@@@@@@@@@##@#@@@@@@#+#@#;,';;::::,,,,,........,;:..,...      
                        `.:';:;'+'+#@#+++##+++@@@###@@@@#;;'+#@@@@@@@@@@@#;#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#@@@@@@@##@@+.#@+:######@###@@@@@@@@@@@##@@@@#@@@@##@+:,;:;::,,,,,..........,;:......`     
                       .;++;:;;'+'##@#+++++';;'+#@@@@@#';;;''###@@@@@@@@@@@+#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#####@@'`##+'######@###@@@@@@@@@@@+@#@@@@@@@#+#@+:::,,,,,..,,....`...`.,;:.......`    
                     `;+#;::;;;+'+#@@#+++'''''''''+++++;;;;'+@#+@@@@@@@@@+@;#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#####@@'.:;;'####@####@#@@@@@@@@@##@#@@#@@@@#+##',:,.....,,,,...``...`.,;:.`.....``   
                    .'++;::;';'+'+#@@+#++;;''''';;;++++;;;'+@@###@@@@@@@@;+++@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@######@@@+.;:.#@##@@######@@@@@@@@@#@#@@@@@@@@#+@#;,;:,.....,,,....``....,::.`.......   
                 ` :+##:::;'';'+;#@@#+#+;;;;''''''+++++';;++##@##@@@@@@@@#:+#+@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@##@##@@@@#++;;@##@#@@###@#@@@@@@@@##@#@@@@@@@@#+@#;,':,,,,,.,,,,,..``....,::.`.....`..` 
                 `:'##:::;'';;++;+#@@##';;;;;''+++++++++;++++#@##@@@@@@@@@@+';@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@,'@@:'#@#@@@@#####@@@@@@@@##@#@@@@@@@@#+@+::'::,,,,,,,,,,,..`...,.::.`.......,:.
                `:'##:::;''';'+';'#@@#+;'';;'+++++'++++++++#''#@+@@@@@@@@@@#:#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#.+@@:+@##@@#@#####@@@@@@@@#@#@@@@@@@@@#+@',:;:,:,,,,,,,,,,......,.:;.`..`.....,;
               `:'+#:::;'''';'+'+++#@+;;;';'++++++'#+++++++#''+@#@@@@@@@@+##:#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@+.:;;;##@#@@#@####@@@@@@@@####@@@@@@@@#+#@',;':,,,,,,,,,,,,,.......::.`..```.,,,:
              `:'+#;::;'''';''+++####''''+++++++++'++++++++####@##@@@@@@@'###'#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#@@@@@:,',:##@@@@#@####@@@@@@@@#@##@@@@@@@@#+##',''::,.,,,,,,,,........,::.`..```.,,,,
             `;;+#;::;;''';'''''###@#+++++++++++++'++++++++#+##@+#@@@@@@@'#@@''@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#:#:+;+@@@@@@#@####@@@@@@@##@#@@#@@@@@@@###;,'';,,,,,,,,,,,.....`..,::.`..````,,,:
            .;'+#'::;;'''';'''';'+##@#++++++++++++'+++++++++++#@##@@@@@@@#;'@++@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@'`@@+.@@@@@@@######@@@@@@@##@###@@@@@@@#+##:,';:,,,,,,,,,,.....,`,.,:,......``.,,:
           `;;+#'::;;;'''''''+';''+@@#++++++++++++++++++++++++#@##@@@@@@@@@+#'#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@;`@@+.@@@@@@@######@@@@@@@+@@@@@@@@@@@@++@+:,':,:,,,,.,,,,....,,.,.:;,.......`.,,,
          `;;+#+::;;;;''''''+#;;'+'#@@++++++++++++++++++++++#++@##@@@@@@@@@@;:###@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@;.+'':@#@@@@@@#####@@@@@@#+@@@@@@@@@@@@##@+,,',,,,,,.,,,,....,,..,.::,.........,,,

"""
