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
        # register_btn = QtGui.QPushButton('Register Points', self)
        self.newPt_btn.setToolTip("Pair the points currently selected in the map")
        self.newPt_btn.clicked.connect(self.register_points)
        # register_btn.resize(register_btn.sizeHint())
        # buttonLayout.addWidget(register_btn)

        # warp_btn = QtGui.QPushButton('Apply Transform', self)
        self.transform_btn.setToolTip("Apply Affine Transform defined by the registered points")
        self.transform_btn.clicked.connect(self.transform_map)
        # warp_btn.resize(warp_btn.sizeHint())
        # buttonLayout.addWidget(warp_btn)

        # Make 3 buttons -- one to edit each point
    ##REPLACE
    ##^^^^^^^
    #Replace this code with a toggle state for the three radio buttons:
        self.point1.clicked.connect(self.change_edit_mode)
        self.point2.clicked.connect(self.change_edit_mode)
        self.point3.clicked.connect(self.change_edit_mode)
        # for i in range(1, 4):
        #     name = "Edit Point " + str(i)
        #     btn = QtGui.QPushButton(name, self)
        #     btn.clicked.connect(self.change_edit_mode)
        #     btn.resize(btn.sizeHint())
        #     buttonLayout.addWidget(btn)

        # Show the layout
        # layout.addLayout(mapLayout)
        # layout.addLayout(buttonLayout)
        # self.setLayout( layout )

        # Set up variables for point registration and transformation, etc
        self.edit_mode = 0
        self.src = [(-1, -1), (-1, -1), (-1, -1)]
        self.dst = [(-1, -1), (-1, -1), (-1, -1)]
        self.robot_on = False
        self.robot = RobotHandler()

    # Add (or remove) a robot from the scene
    def robot_toggle(self):
        if not self.robot_on:
            self.robot.setEnabled(True)
            self.robot_on = True
        else:
            self.robot.setEnabled(False)
            self.robot_on = False

    # Edit a different point
    def change_edit_mode(self):
        sender_name = self.sender().text()
        if sender_name == "point1":
            self.edit_mode = 1
        elif sender_name == "point2":
            self.edit_mode = 2
        elif sender_name == "point3":
            self.edit_mode = 3
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
            cv2.imshow('Output', output)
        else:
            print "Not enough pairs to transform"

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