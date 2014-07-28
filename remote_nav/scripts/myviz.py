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

## Import all the Qt bindings into the current namespace, for
## convenience. This uses the "python_qt_binding" package which hides
## differences between PyQt and PySide, and works if at least one of
## the two is installed. The RViz Python bindings use
## python_qt_binding internally, so you should use it here as well.
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

## Finally import the RViz bindings themselves.
import rviz

#Get moving!
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, PointStamped
from actionlib_msgs.msg import GoalID
# from pr2_controllers_msgs.msg import PointHeadActionGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
import actionlib
#tf for more navigation stuff
import tf
#rospkg is to handle possible pathing issues when finding images and configuration files.
import rospkg


## The MyViz class is the main container widget.
class MyViz( QWidget ):

	## MyViz Constructor
	def __init__(self):

		QWidget.__init__(self)
		self.is_pr2 = rospy.get_param("remote_nav/is_pr2", False)
		self.init_ros_variables()
		
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
		config_file = rospy.get_param('remote_nav/rviz_config', package_path + "/rviz/turtle_map_img.rviz")
		reader.readFile( config, config_file )
		self.frame.load( config )

		self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )
		self.setWindowIcon(QIcon(package_path +'/images/icon.png'))


	#Disable unneeded views and more visualization setup
		self.frame.setMenuBar( None )
		self.frame.setStatusBar( None )
		self.frame.setHideButtonVisibility( False )
		self.manager = self.frame.getManager()
		self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
		
	##LAYOUT
	##^^^^^^
		layout = QGridLayout()
		layout.setSpacing(10)

		layout.addWidget(self.frame, 1, 0, 4, 3)
		
	 	h_layout = QHBoxLayout()
		
	#Buttons and attached commands
		# 1. Create Button
		# 2. Connect Signal to Slot
		# 3. Add to layout

		self.look_fwd_button = QPushButton( "Look Forward" )
		self.look_fwd_button.clicked.connect( self.onLookFwdButtonClick )
		self.look_fwd_button.setToolTip('Press this to look forward.')
		self.look_fwd_button.setStyleSheet("background-color: #700000 ; font-weight: bold; color: white")
		layout.addWidget( self.look_fwd_button, 8, 1 )
		
		self.fwd_button = PicButton(QPixmap(package_path + "/images/up.png"))
		self.fwd_button.setClickPix(QPixmap(package_path + "/images/upDark.png"))
		# self.fwd_button = QPushButton("Move Forward")
		self.fwd_button.pressed.connect( self.onFwdPress )
		self.fwd_button.setToolTip('While held, the robot will move forward')
		layout.addWidget( self.fwd_button, 5, 1 )
		layout.setAlignment(self.fwd_button, Qt.AlignHCenter)

		turn_button = PicButton(QPixmap(package_path + "/images/rotate.png"))
		turn_button.setClickPix(QPixmap(package_path + "/images/rotateDark.png"))
		turn_button.clicked.connect( self.onTurnButtonClick )
		turn_button.setToolTip('The robot will turn around 180 degrees')

		self.look_left_btn = PicButton(QPixmap(package_path + "/images/left.png"))
		self.look_left_btn.setClickPix(QPixmap(package_path + "/images/leftDark.png"))

		self.look_right_btn = PicButton(QPixmap(package_path + "/images/right.png"))
		self.look_right_btn.setClickPix(QPixmap(package_path + "/images/rightDark.png"))

		# Only actually connect these to the moving of the head if there is in fact a head.
		if (self.is_pr2):
			self.look_left_btn.pressed.connect( self.onLeftButtonClick )
			self.look_right_btn.pressed.connect( self.onRightButtonClick )
		

		#Finalizing layout and placing components
		h_layout.addWidget(self.look_left_btn)
		h_layout.setAlignment(self.look_left_btn, Qt.AlignRight)
		h_layout.addWidget(turn_button)
		h_layout.setAlignment(turn_button, Qt.AlignHCenter)
		h_layout.addWidget(self.look_right_btn)
		h_layout.setAlignment(self.look_right_btn, Qt.AlignLeft)

		layout.addLayout( h_layout, 7, 1 )	
		self.setLayout( layout )

	

	#Initialize ROS variables, listeners, publishers, etc. Here.
	def init_ros_variables(self):
	#For sending nav goals.
		nav_topic = rospy.get_param("remote_nav/nav_topic", "/move_base_simple/goal")
		self.nav_pub = rospy.Publisher(nav_topic, PoseStamped)
	#A publisher to literally tell dis bisnatch to cancel all goals.
		cancel_topic = rospy.get_param("remote_nav/cancel_topic", '/move_base/cancel')
		self.cancel_pub = rospy.Publisher(cancel_topic, GoalID)

	#We choose in our implementation to move the head using the preexisting head trajectory controller.
		if (self.is_pr2):
			head_server = rospy.get_param("remote_nav/head_server", 'are you a turtlebot? This no for turtlebot')
			self.client = actionlib.SimpleActionClient(head_server, PointHeadAction)
			self.client.wait_for_server()	
	#We need be transformin these mofuckin frames.
		self.listener = tf.TransformListener()

	#Is the robot facing forward along our track?
		self.isForward = True
	#Get the track_length for our 1D start track.
		self.track_length = rospy.get_param('remote_nav/track_length', 5.0)
		self.robot_frame = rospy.get_param('remote_nav/robot_frame', "/base_footprint")
	

## Handle GUI events
## ^^^^^^^^^^^^^^^^^

	def closeEvent(self, event):

		reply = QMessageBox.question(self, 'Message',
		"Are you sure you want to quit?", QMessageBox.Yes | 
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


	def onTurnButtonClick(self):
		if self.isForward:
			self.faceBackward()
		else:
			self.faceForward()

	def onResetDirButtonClick(self):
		self.faceForward()

	def onLeftButtonClick(self):
		self.lookLeft()

	def onRightButtonClick(self):
		self.lookRight()

	def onLookFwdButtonClick(self):
		QApplication.processEvents()
		self.lookFwd()

## MOVE ZE HEAD FUNCTIONS
## ^^^^^^^^^^^^^^^^^^^^

	def lookLeft(self):
		parent_frame = "base_link"
		x = 0.0
		y = 1.5
		z = 1.2

		self._look_at(parent_frame, x, y, z)
		while self.look_left_btn.isDown():
			QApplication.processEvents()
		self.client.cancel_all_goals()


	def lookRight(self):
		parent_frame = "base_link"
		x = 0.0
		y = -1.5
		z = 1.2
		self._look_at(parent_frame, x, y, z)
		while self.look_right_btn.isDown():
			QApplication.processEvents()
		self.client.cancel_all_goals()
	def lookFwd(self):
		parent_frame = "base_link"
		x = 1.0
		y = 0.0
		z = 1.2

		self._look_at(parent_frame, x, y, z)


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

	#Cancels the current nav goal
	def _cancel_goals(self):
		goalID = GoalID()
		self.cancel_pub.publish(goalID)

	#Sets a 3D goal for the head of the PR2 to look. Do not use with turtlebot.
	def _look_at(self, parent_frame, x, y, z):
		goal = PointHeadGoal()
		#the point to be looking at is expressed in the "base_link" frame
		point = PointStamped()
		point.header.frame_id = parent_frame
		point.header.stamp = rospy.Time.now()
		point.point.x = x
		point.point.y = y 
		point.point.z = z
		goal.target = point

		#we want the X axis of the camera frame to be pointing at the target
		goal.pointing_frame = "high_def_frame"
		goal.pointing_axis.x = 1
		goal.pointing_axis.y = 0
		goal.pointing_axis.z = 0
		# goal.min_duration = rospy.Duration(2.0)
		goal.max_velocity = 0.85
		self.client.send_goal(goal)


#Unique UI Classes
#^^^^^^^^^^^^^^^^

#This class makes a button that is a picture instead of text.  
#After initializing, passing in a QPixmap, you can call setClickPix to add an onClick image.
class PicButton(QAbstractButton):
	def __init__(self, pixmap, parent=None):
		super(PicButton, self).__init__(parent)
		self.pixmap = pixmap
		self.clickpix = pixmap

		#If the image may end up too large, scale it down to 50x50.
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
                                                                                                       ,:::,:,:::::,,:,,,,,,.                                                                                  
                                                                                           ``     ``,::,::,,::::::::,,,,,,,,,,..                                                                              
                                                                                            `   `.,:::::::::::::::::,,,,,,,,,,,,,,                                                                             
                                                                                            ```.::::::::::::::::::::,,,,,,,,,,,,,,,,                                                                         
                                                                                           ` `,::::::::::;::;:::::::,,,,,,,,,,,,,,,,,.                                                                        
                                                                                            .:;::::;;;;;;;;;::::::::,,,,,,,,,,,,,,,,,,,                                                                      
                                                                                      ```  .;;;;;;;;;;;;;:;;::::::::::,,,,,,,,,,,,,,,,,,.                                                                     
                                                                                      `   .;::;;;;;;;;;;;:;;;;::::::::::::,,,,,,,,,,,,,,,.                                                                     
                                                                                       ` ,::::;;;;;;;;;;;:;::::::::::::::::,,,,,,,,,,,,,,,.                                                                   
                                                                                        .::;;;;;;;;;;;;;;;;;;;;;;;;;;:::::::,,,,,,,,,,,,,,,.                                                                   
                                                                                       .;:;;;;;;;;;;;;;;;;;;;;;;;;;;;::::::::,,,,,,,,,,,,,,,`                                                                
                                                                                   `` .:;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;:::::::,,,,,,,,,,,,,,,`                                                               
                                                                                 `   `:;;;;;;'''';;;;;;;;;;;;;;';;;;;;;;;::::,:,,,,,,,,,,,,,,,`                                                               
                                                                                `    ,;;;;'''''';;;;;;'';;;;;;;''';;;;;;;::::,,,,:,,,,,,,,,,,,.                                                              
                                                                                    .;;;;;'''''';;;;;;'''''''''''';;;;;;;:::::,,:,,,,,,,,,,,,,,.                                                               
                                                                                 `  :;;;;''''''';;''''''''''''''''';;;;;;;::,,,,,,,,,,,,,,,,,,,,`                                                              
                                                                                 ` `;;;'''''''''''''''''''''''''''';;;;;:;:::,,,,,,,,,,,,,,,,,,,.`                                                            
                                                                                  `:;;;''''''''''''''''''''''''''''';;;;;;:::,,,,,,,,,,,,,,,,,,,,.                                                             
                                                                                  `;;;;''''''''''''''''''''''''''''';;;;;;:::,,,,,,::,,,,,,,,,,,,,`                                                           
                                                                                  .;;;''''+'''''''+'''''++''''''''''';';;;::,,,,,,,,:,,,,,,,,,,,,,.                                                          
                                                                                ` :;';'''''''''''+++++''+'''''''''''';;;;;;:,,,,,,,,,,::,,,,,,,,,,,`                                                          
                                                                               ` `:;;''''''''''''++'++++''+++''''''''';;;;;::,,,,,,,,,,:,,,,,,,,,,,.                                                           
                                                                             `   .;;;''''''''''''+''+++++''++''''''''';;;;;:::,,,,,,,:,,,,,,:,,,,,,,                                                          
                                                                                 ,;;;'''''''''''++'+++++++'++''''''''';;;;;:::,,,,,,,,,,,,,,:,,,,,,,`                                                          
                                                                                `;;;;'''''''''''++'++++++++++++'''''''';;;::::,,,,,,,,,,,,,,:::,,,,,.                                                         
                                                                              ` .;;;;'''''''''''''+++++++++'++++'''''''';;::::,,,,,,,,,,,,,,::::,,,,,                                                          
                                                                                ,;;;;''''''''''''''+++++++++++++'''''''';;::::,,,.,,,,,,,,:,::::,,,,,`                                                         
                                                                               `,;;;;'''''''''''''++++++++++++++''''''';;;::::,,,,,,,,,,,:::::::,,,,,`                                                         
                                                                               `:;;;;''''''''''''''+++++++++++++'''''''';;:::::,,,,,,,,,,:::::::,,,,,.                                                         
                                                                            `  .:;;;;''''''''''''''+++++++++++++'''''''';;:::::,,,,,,,,:,:::::::,,,,,.                                                        
                                                                               .;;;;;''''''''''''''++++++++++++++''''';;;;:::::::,,,,,,:::::::::,,,,,,                                                       
                                                                               ,;';;;'''''''''''''''+++++++++++++'''';;;;;;:::::::,,,:::::::;;::,,,,,,                                                         
                                                                               :;';;;'''''''''''''''+++++++++++++'''';;;;;;;::,::,:,,:,,::::;;::,:::,.                                                         
                                                                              `:;;;;;';''''''''''''''++++++++++++'''';;';;;;:::::::,::::::;;;;::::,,,.                                                        
                                                                      ` `     `:;;;;;;;'''''''''''''''+++++++++++''''';'';;;:::::::::::::;:;;;;::::,,.                                                         
                                                                              .:;;;;;;'''''''''''''''''+++++++++++'''';;';;;:::::::::::::;::;;;:::::,.                                                         
                                                                         `    .;;;;;;;'''''''''''''''''+++++++++++''''''';;;;::::::::::;;;;;;;;:;;::,.                                                         
                                                                      `       ,;;;;;;;'''''''''''''''''+++++++++++''';;';;;;;;::::::;::;;;;;;;;;:;:::.                                                         
                                                                       `:,`   :';';;:;'''''''''''''''''''+++++++++''''''';;;;:;::;::::;;;;;;;;;;;;:::.                                                         
                                                                     ``;+#'  `;'';;;::''''''''''+++'''''''++++++++''''''';;;;;:;;;;;;;;;;;;;;;:;;::::.                                                         
                                                                      .;'#@:``;''';;:;''''''''''+++++''''''+++++++''''';';;;;;;;;;;;;;;;;;;;;:;;;::::`                                                         
                                                                      '#'+##``'''';;;;''''''''''+++++'''''+++++++++'''''';;;;;;;;;;;;;;;;;;;;;;:;::,,`                                                         
                                                                     .###'##@;'''';:;''''''''''''++++++'''+++++++++''';';;;;;;;;;;;;;;;;;;;;;;;;;::,,`                                                         
                                                                     :##+'####+';';;;'''''''''''+++++++++++++++++++'''';;;;;;;;;;;;;;;;;;;;;;;;;:::,,                                                        
                                                                     ;##''+###+';;;;''''''''''''++++++++++++++++++++''';;;;;;;;;;'';;;;;;;;;;;;;::,,.                                                          
                                                                     ;@+'''+#++';;;;''''''''''++'+++++++++++++++++++''';;;;;;;;;'''';;;;;;;;;;;;,,,,`                                                          
                                                                     ;#'''''+++';;;;'''''''''''''+++++++++++++++++++'';;;;;;;;;;;;;;;;;;;;;;;;;:,:,,                                                           
                                                                     :+'++'''''';;;''''''''++++'++'++++++++++++++++''';;;;;;;;;;;;;;;;;;;;;;;;;,,,:,                                                          
                                                                     :+++++''''+;;;'''''+++++++++'++++++++++++++++''''';;;;;;:::;;::;;;;;;;;;;:,:,:.                                                          
                                                                     :+++##+'''+;;;''''+++++++++++''++++++++++++++'''''';;;;;;::::::;;;;;;;;;;,,::,`                                                          
                                                                     ;####+++'+;;;''''''''++#+++++++++++++'''++++''''''';;;;:::::::::;;;;;;;;:,,,,,`                                                          
                                                                    `'####+'+'';;;''';;''''''++##++++++''+'''''''''''''';;;;:::::::::;;;;;;;;:,:::.                                                           
                                                                    `'##@@+'++';;''''';;'''''''+###+++++'''''''''''''''';;;::::::::::;;;;;;';:,,:,`                                                            
                                                                `   `'#@@@+''';;;'''''''''+++''''++##+++''''''''''''''';;';;:::::::::;;;;;;;:,,::.                                                           
                                                                    `'+#@@+'';;'''''''''+########++++#+++''''''''''''';;';;;::::::::::;;;;;;:,:::`                                                             
                                                                     ;''+@''';''''''''''+++++###@@@##+++'''''''''''''''''';;;:::::::,:::;;;;:,::,                                                              
                                                                     :#'+#'';'''''''++'''#@##+@@@##@@#++++''''''';;;;;;;'';;;:::::,,::::;;;;:,::`                                                        
                                                                     .#++@#';''''''++++'''+#,.'#@@@##@#++''''''''';;;;'''';;;::;::::::::;;;;:::,                                                          
                                                                     `+++@#;'''''''++++++''#'`;++;+###@#+'''''''''';;'''''''''''';;;;;:::;;;:::` .;,,                                                         
                                                                      ;+'++;'''''''++++#''''#;,'+++:+###++''''''';;;''''''+++++++'''';;;::;;::;:;;,,:.                                                       
                                                                      .+''+'''''''''+++#+'';'##+++'+#####+''''''';;;'''+++##+++#++++'''';;;;:::,:;;::`                                                         
                                                                     ` ;''+;''''''''+++++''';''#@@@@@@###+''''''';;''''+++####++'+''''''';;;::::;''#'                                                         
                                                                       `;'+;'''''''''++++'''''''++++#####+''''''';;'''+'+@@@@@###+''''''';;;,;;;''#@:                                                          
                                                                        `'+;''+'''''''++++''''++++++####++''''''''''+++#@@#####+++##+''';;;;:';'''##.                                                          
                                                                         .';'''''';''''+++++'''+++++##+#+++''++'''''++####'@@@@@##++#++';;;::';'';++                                                          
                                                                        ` .;'''''''''''++++++++++++++#+++++''+++';'+++#+##;##+;'+'@#+#++';;::'''';+;                                                          
                                                                        ` ,'''''''';''''+++++++++++++++++++++'+';:''+++##@#++'':.,+@#+++'';::'+++'+.                                                           
                                                                          ,';'''''';;''''++++++++++++++++++'''+';,;'++++####@+;..'+##++''';::#+++';                                                            
                                                                          ,';''''''';'''''+++++++++'''+++++''++';,:;++++##++++';''''''''';;::#+++'.                                                           
                                                                       `  :'''''''';''''''++++++++''''''+++''+'':,:;'+++++++''''''';;;;;;;::,++++;                                                             
                                                                       `  :'''''''';''''''+'+++++'''''''''''++';:,::;'++++++'''''';;::::;;:::#++',                                                            
                                                                          ;'''''''''''''''''''+++'''''''++''++';::::;;''++'+''''';;:,,,::::::+;;'`                                                             
                                                                         `;'''''''''''''''''''+++'''''''++++++';::::;;;;''''''';;::,,,,,;;::::,;:                                                              
                                                                         `;'''''''''''';'''''''+++''''''++++++':,::::;;;;;''''';::,,,,,::::,,:':                                                             
                                                                         `;'''''''''''';'''''''+++'''''''+++'';,,,,::;;;;;;;';;;::,,,,:::;::,''`                                                               
                                                                         .'''''''''''''''''''''+++''++''++++';:,.,,,:;;;:;:;;;;;:::,:::;;;::,:.                                                               
                                                                     `   .''''''''''';''''''''''++'++++''+++';:..,,,::;;:::::;;;:;::::;;;;:::,`                                                                
                                                                         ,+'''''''''''''''''''''+'''+++''+++';:,..,,:;;;;;;:;;;;;;;;;;;;;;:;:.                                                                 
                                                                         ,+'''''''''''''''''''''+'''+++''++++;:,,,,,,;'';;;;:;;;';;;;;;;;;::,`                                                                 
                                                                       ` ,+'''''''''''''''''''+++''''''''++++':,,:,,,;+';;;;;;;;;;;;'''';:::,                                                                  
                                                                         ,'''''''''''''''''''+++++''''''''+++';:::::,;+''';;;;;;;;;''''';::`.                                                                 
                                                                         ,+''''''''''''''''''+++++'''''';''''';;;;;::;++'';;;;;;;''''''';;, .                                                                  
                                                                         .+''''''''''''''''''++++##+'+++''''';;;;;;;;;++''';;;;;'''''''';'`,,                                                                  
                                                                         .''''''''''''''''''+++++++#+''''''';;''+'''''++'''''';;'''''''':+,,                                                                   
                                                                         .+'''''''''''''''''++++++++++++#+''''++++++'''+''''''''''''''''``.`                                                                   
                                                                       ` `'''''''''''''''''++++++++++#####+'+++++++';''++'''''''''''''+:                                                                       
                                                                          ''''''''''''''''++++++++++++##+++++++++';;;''++'''''''''++++'`                                                                      
                                                                          ;+'''''''''''''''+++++++++++##+++++++++;;;;;''++''''''++++++;                                                                       
                                                                        ` .+''''''''''''''''++'++#++++#+#+++++++';;;;;'''''''''+++++++.                                                                       
                                                                          `''''''''''''''''++############+###+'';;;;;''''''''++++++++'`                                                                        
                                                                           ,''''''+'''''''+++################++'';'''';''''++++++++++,                                                                         
                                                                           .'''''++''''''++++++############@#+++''';;';''''+++++++++'`                                                                       
                                                                           `+''''''''''''+''''''+++++##+##@@##+#++'';'''''+++++++++++;:,.`                                                                     
                                                                 `   `      :+'''''''+''+'+++';''''''''+++++'''++#+'''''''++'++++++++##@##+':.                                                                
                                                                    `  `.:'##'''''''''''+'++@@#+''''''++'''''';;;'+++'''''++++++++++.###@@@@@@#+;,.                                                          
                                                                 `   ,;+#@@@@#''''''+'''+'''+@####+''''''''''++';;'+#++'''+++++++++'.:@##@@@@@@@@@##+;,.                                                      
                                                               `  `,#@@@@@@@@#+'''''+''+'''''#+'+####+''''''''''+';+++++'++++++++++;;.+@++#@@@@@@@@@@@@##+';,                                              
                                                                 ,+@@@@@@@@@@#++'''''+++''''''+''++#########+++++++++##+++++++++++;;#;,##++#@@@@@@@@@@@@@@@@@##+'::.                                          
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
