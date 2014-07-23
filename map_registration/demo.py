import sys, getopt
from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import * 
import numpy as np
from components import * 

from PyQt4.QtGui import QDialog
from mapTransform import Ui_Window

class MainWindow(QDialog, Ui_Window):
    def __init__(self, map1, map2, parent=None):
        super(QDialog, self).__init__(parent)
        self.setupUi(self)
        self.setWindowTitle('Main Window')
 
        # Sets up the maps
        self.img_1 = map1
        self.img_2 = map2
        self.map1 = DrawMap(self.img_1, self)
        self.source.setScene( self.map1 )
        self.map2 = DrawMap(self.img_2, self)
        self.destination.setScene( self.map2 )

        # Set up variables for point registration and transformation, etc
        self.edit_mode = 0
        self.src = [(-1, -1), (-1, -1), (-1, -1)]
        self.dst = [(-1, -1), (-1, -1), (-1, -1)]
        robot1 = DrawRobot()
        robot2 = DrawRobot()
        robot1.setVisible(False)
        robot2.setVisible(False)
        self.robot = RobotHandler(robot1, robot2)

        #Changes GUI attributes

        self.map1.addItem(robot1)
        self.map2.addItem(robot2)


        self.bulge_btn.setIcon(QIcon("images/bulge.png"))
        self.indent_btn.setIcon(QIcon("images/indent.png"))

        self.transform_btn.setToolTip("Apply Affine Transform defined by the registered points")
        self.transform_btn.clicked.connect(self.transform_map)

        self.export_btn.setToolTip("Save the transformed map")
        self.export_btn.clicked.connect(self.export_map)


    ##SIGNALS AND SLOTS
    ##^^^^^^^^^^^^^^^^^
        # The signals are emitted after a click in the map window
        self.map1.register.connect(self.point_handler)
        self.map2.register.connect(self.point_handler)

        #Setting up the robot toggled checkbox
        self.toggleRobot.stateChanged.connect(self.robot_toggle)

        self.change_edit_mode()
        self.point1.toggled.connect(self.change_edit_mode)
        self.point2.toggled.connect(self.change_edit_mode)
        self.point3.toggled.connect(self.change_edit_mode)



    # Add (or remove) a robot from the scene
    def robot_toggle(self):
        self.robot.setEnabled(self.toggleRobot.isChecked())

    # Edit a different point
    def change_edit_mode(self):
    ##Each of the buttons is associated with an ID number, and we can use this to set the mode.
        buttonId = self.buttonGroup.checkedId()
        if buttonId <= 0:
            print ("No button selected!")
        else:
            self.edit_mode = buttonId
            if self.src[buttonId - 1] == (-1, -1):
                self.map_1_set = False
            if self.dst[buttonId - 1] == (-1, -1):
                self.map_2_set = False
            self.map1.change_edit_mode(self.edit_mode)
            self.map2.change_edit_mode(self.edit_mode)

    # Using registred points, transform the maps
    def transform_map(self):
        # Check that three pairs have been make
        if ((-1, -1) not in self.src) and ((-1, -1) not in self.dst):
            print "Transforming Maps"
            # Turn the pairs into an Affine Transformation matrix
            numpy_src = np.array(self.src, dtype='float32')
            numpy_dst = np.array(self.dst, dtype='float32')
            transform = self.robot.setTransforms(numpy_src, numpy_dst)
            # Apply the transform 
            if transform != None:
                src = cv2.imread(self.img_1, 0)
                rows, cols = src.shape
                output = cv2.warpAffine(src, transform, (cols, rows))
                self.outputWindow(output)
            # cv2.imshow('Output', output)
        else:
            print "Not enough pairs to transform"

    def export_map(self):
        pass

    # Reads the most recent points off the maps and puts into the Matrix
    def register_points(self):
        print "Registering Points"
        if self.edit_mode != 0:
            self.src[self.edit_mode - 1] = self.map1.getPoint()
            self.dst[self.edit_mode - 1] = self.map2.getPoint()
            # print "Source: ", self.src
            # print "Destination: ", self.dst
    
    # triggered when a point is clicked in either map scene
    def point_handler(self):
        if self.sender() is self.map1:
            # Update the label
            pos = self.map1.getPoint()
            if self.edit_mode == 1:
                self.p1_x1.setText(str(pos[0]))
                self.p1_y1.setText(str(pos[1]))
            elif  self.edit_mode == 2:
                self.p2_x1.setText(str(pos[0]))
                self.p2_y1.setText(str(pos[1]))
            elif self.edit_mode == 3:
                self.p3_x1.setText(str(pos[0]))
                self.p3_y1.setText(str(pos[1]))
            else:
                print "Not currently editing a point"
            self.register_points()
        elif self.sender() is self.map2:
            # update the label
            pos = self.map2.getPoint()
            if self.edit_mode == 1:
                self.p1_x2.setText(str(pos[0]))
                self.p1_y2.setText(str(pos[1]))
            elif  self.edit_mode == 2:
                self.p2_x2.setText(str(pos[0]))
                self.p2_y2.setText(str(pos[1]))
            elif self.edit_mode == 3:
                self.p3_x2.setText(str(pos[0]))
                self.p3_y2.setText(str(pos[1]))
            else:
                print "Not currently editing a point"
            self.register_points()
        else:
            print "Unknown event trigger"

    def outputWindow(self, image):
        cv2.imshow('Preview', image)
        # child = MyWindow(image, self)
        # child.show()

class MyWindow(QtGui.QDialog):    # any super class is okay
    def __init__(self, image, parent=None):
        super(MyWindow, self).__init__(parent)
        self.export = QtGui.QPushButton('Export')

        self.pic = QtGui.QLabel()
        self.pic.setGeometry(10, 10, 100, 400)
        #use full ABSOLUTE path to the image, not relative
        self.pic.setPixmap(QtGui.QPixmap(image))

        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.pic)
        layout.addWidget(self.export)
        self.setLayout(layout)
        self.export.clicked.connect(self.export_image)
    def export_image(self):
        pass

def main(argv):
    usage = "demo.py <source image> <destination image>"
    src = ""
    dst = ""

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

    src = args[0]
    dst = args[1]

    print src, dst

    if src == "" and dst == "":
        print usage 
        sys.exit(2)

    app = QApplication( sys.argv )
    mainWindow = MainWindow(src, dst)
    mainWindow.resize( 1000, 500 )
    mainWindow.show()
    app.exec_()

if __name__ == '__main__':
    main(sys.argv[1:])