import sys, getopt
from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import * 
import cv2
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

        # BUTTONS
        self.newPt_btn.setToolTip("Pair the points currently selected in the map")
        self.newPt_btn.clicked.connect(self.register_points)

        self.transform_btn.setToolTip("Apply Affine Transform defined by the registered points")
        self.transform_btn.clicked.connect(self.transform_map)

        self.export_btn.setToolTip("Save the transformed map")
        self.export_btn.clicked.connect(self.export_map)

        # The signals are emitted after a click in the map window
        self.map1.register.connect(self.register_points)
        self.map2.register.connect(self.register_points)

        #Setting up the robot toggled checkbox
        self.toggleRobot.stateChanged.connect(self.robot_toggle)

        # Make 3 buttons -- one to edit each point
    ##REPLACE
    ##^^^^^^^
    #Replace this code with a toggle state for the three radio buttons:
        self.point1.toggled.connect(self.change_edit_mode)
        self.point2.toggled.connect(self.change_edit_mode)
        self.point3.toggled.connect(self.change_edit_mode)
        # for i in range(1, 4):
        #     name = "Edit Point " + str(i)
        #     btn = QtGui.QPushButton(name, self)
        #     btn.clicked.connect(self.change_edit_mode)
        #     btn.resize(btn.sizeHint())
        #     buttonLayout.addWidget(btn)


        # Set up variables for point registration and transformation, etc
        self.edit_mode = 0
        self.src = [(-1, -1), (-1, -1), (-1, -1)]
        self.dst = [(-1, -1), (-1, -1), (-1, -1)]
        self.robot_on = False
        self.robot = RobotHandler()

    # Add (or remove) a robot from the scene
    def robot_toggle(self):
        # if not self.robot_on:
        self.robot.setEnabled(self.toggleRobot.isChecked())
        self.robot_on = self.toggleRobot.isChecked()
        print self.toggleRobot.isChecked()
        # else:
        #     self.robot.setEnabled(False)
        #     self.robot_on = False

    # Edit a different point
    def change_edit_mode(self):
    ##Each of the buttons is associated with an ID number, and we can use this to set the mode.
        buttonId = self.buttonGroup.checkedId()
        self.edit_mode = buttonId
        if buttonId <= 0:
            print ("No button selected!")
        else:
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
            transform = cv2.getAffineTransform(numpy_src, numpy_dst)
            print transform
            # Apply the transform 
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
        # check that both map1.getPoint() and map2.getPoint() have been set
        if (self.map1.getPoint() != (-1, -1)) and (self.map2.getPoint() != (-1, -1)):
            print "Registering Points"
            if self.edit_mode != 0:
                self.src[self.edit_mode - 1] = self.map1.getPoint()
                self.dst[self.edit_mode - 1] = self.map2.getPoint()
                print "Source: ", self.src
                print "Destination: ", self.dst
        else:
            print "Not all points have been set"
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