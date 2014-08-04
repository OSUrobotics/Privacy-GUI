import sys, getopt
from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import * 
import numpy as np
from components import * 
from itertools import izip

from PyQt4.QtGui import QDialog
from mapTransform import Ui_Window
from os import path
from subprocess import call

class MainWindow(QDialog, Ui_Window):
    def __init__(self, semantic, slam, parent=None):
        super(QDialog, self).__init__(parent)
        self.setupUi(self)
        self.setWindowTitle('Main Window')
 
        # Sets up the maps
        self.img_1 = semantic
        slam_meta_data = yaml_to_meta_data(slam)
        self.slam_origin = slam_meta_data.origin
        self.slam_res = slam_meta_data.resolution
        self.img_2 = path.dirname(slam) + "/" + slam_meta_data.image
        self.map1 = DrawMap(self.img_1, self)
        self.source.setScene( self.map1 )
        self.map2 = DrawMap(self.img_2, self)
        self.destination.setScene( self.map2 )

        # Set up variables for point registration and transformation, etc
        self.src = [(-1, -1), (-1, -1), (-1, -1)]
        self.dst = [(-1, -1), (-1, -1), (-1, -1)]
        self.robot1 = DrawRobot()
        self.robot2 = DrawRobot()
        self.robot1.setVisible(False)
        self.robot2.setVisible(False)
        self.robot = RobotHandler(self.robot1, self.robot2)

        #Changes GUI attributes
        self.toggleRobot.setEnabled(False)

        self.map1.addItem(self.robot1)
        self.map2.addItem(self.robot2)

        # self.point1.setStyleSheet("color:red") 
        self.bulge_btn.setIcon(QIcon("images/bulge.png"))
        self.indent_btn.setIcon(QIcon("images/indent.png"))

        self.transform_btn.setToolTip("Apply Transform and show the result")
        self.transform_btn.clicked.connect(self.transform_map)

        self.export_btn.setToolTip("Save the transformed map")
        self.export_btn.clicked.connect(self.export_map)
        self.update_labels()

        # self.newPt_btn.setToolTip("Applies the transform defined by the current points")
        # self.newPt_btn.clicked.connect(self.transform_map)

    ##SIGNALS AND SLOTS
    ##^^^^^^^^^^^^^^^^^
        # The signals are emitted after a click in the map window
        self.map1.register.connect(self.point_handler)
        self.map2.register.connect(self.point_handler)

        #Setting up the robot toggled checkbox
        # self.toggleRobot.stateChanged.connect(self.robot_toggle)

    # Updates the labels telling how many points there are
    def update_labels(self):
        self.label_4.setText(str(self.map1.get_num_points()))
        self.label_5.setText(str(self.map2.get_num_points()))

    # Add (or remove) a robot from the scene
    def robot_toggle(self):
        if self.toggleRobot.isChecked():
            self.transform_array()
        self.robot.setEnabled(self.toggleRobot.isChecked())

    # Using registred points, transform the maps
    def transform_map(self):
        self.register_points()
        # Check that three pairs have been make
        if ((-1, -1) not in self.src) and ((-1, -1) not in self.dst):
            print "Transforming Maps"
            self.transform_array()
            # Apply the transform 
            if self.transform != None:
                src = cv2.imread(self.img_1, 0)
                rows, cols = src.shape
                output = cv2.warpAffine(src, self.transform, (cols, rows))
                if self.sender() is self.transform_btn:
                    self.outputWindow(output)
            # cv2.imshow('Output', output)
        else:
            print "Not enough pairs to transform"

    #Turn point pairs into an affine transformation matrix and pass to the robot handler
    def transform_array(self):
        # Turn the pairs into an Affine Transformation matrix
        numpy_src = np.array(self.src, dtype='float32')
        numpy_dst = np.array(self.dst, dtype='float32')
        self.transform = self.robot.setTransforms(numpy_src, numpy_dst)

    # Saves the values in a yaml file in the current directory
    def export_map(self):
        num_pts = self.register_points()
        semantic = str(num_pts) + " 2 0\n"
        slam = str(num_pts) + " 2 0\n"
        semantic += "# Nodes:\n"
        slam += "# Nodes:\n"
        counter = 0
        for p1, p2 in izip(self.map1.get_points(), self.map2.get_points()):
            if p1 != None and p2 != None:
                counter += 1
                semantic += str(counter) + " " + str(p1[0]) + " " + str(p1[1]) + "\n"
                slam += str(counter) + " " + str(p2[0]) + " " + str(p2[1]) + "\n"
        f1 = open('semantic.node', 'w')
        f2 = open('slam.node', 'w')
        f1.write(semantic)
        f2.write(slam)
        f1.close()
        f2.close()
        print semantic
        print slam
        # Triangulate the points
        call(["./triangle/triangle", "semantic.node"])
        call(["./triangle/triangle", "slam.node"])

        # Write the file that relates the two maps. 
        yaml = "semantic_map: " + path.basename(self.img_1) + "\n"
        yaml += "slam_map: " + path.basename(self.img_2) + "\n"
        yaml += "origin: " + str(self.slam_origin) + "\n" 
        yaml += "resolution: " + str(self.slam_res) + "\n"
        src = cv2.imread(self.img_1, 0)
        dst = cv2.imread(self.img_2, 0)
        src_rows, src_cols = src.shape
        dst_rows, dst_cols = dst.shape
        yaml += "slam_width: " + str(dst_cols) + "\n"
        yaml += "slam_height: " + str(dst_rows) + "\n"
        yaml += "semantic_width: " + str(src_cols) + "\n"
        yaml += "semantic_height: " + str(src_rows) + "\n"
        yaml += "semantic_to_slam:\n"
        yaml += "slam_to_semantic:\n"
        f = open('registration.yaml', 'w')
        f.write(yaml)
        f.close()

        # Make a new directory to hold the output. It's okay if it fails,
        # because that means the directory already exists and it will just 
        # overwrite the stuff already there. 
        call(["mkdir", "register/"])
        # Move everything into register'd folder. 
        # call santises inputs so we can't use wildcards
        call(["mv", "semantic.node", "register/"])
        call(["mv", "semantic.1.ele", "register/"])
        call(["mv", "semantic.1.node", "register/"])
        call(["mv", "slam.node", "register/"])
        call(["mv", "slam.1.ele", "register/"])
        call(["mv", "slam.1.node", "register/"])
        call(["mv", "registration.yaml", "register/"])
        call(["cp", self.img_1, "register/"])
        call(["cp", self.img_2, "register/"])

    def printable_1_to_2(self):
        self.transform_array()
        trans = self.robot.trans_1_to_2
        transform = "\"[[ " + '{:e}'.format(trans[0][0]) + " " + '{:e}'.format(trans[0][1]) + " " + '{:e}'.format(trans[0][2]) + " ] "
        transform += "[ " + '{:e}'.format(trans[1][0]) + " " + '{:e}'.format(trans[1][1]) + " " + '{:e}'.format(trans[1][2]) + " ]]\""
        return transform

    def printable_2_to_1(self):
        self.transform_array()
        trans = self.robot.trans_2_to_1
        transform = "\"[[ " + '{:e}'.format(trans[0][0]) + " " + '{:e}'.format(trans[0][1]) + " " + '{:e}'.format(trans[0][2]) + " ] "
        transform += "[ " + '{:e}'.format(trans[1][0]) + " " + '{:e}'.format(trans[1][1]) + " " + '{:e}'.format(trans[1][2]) + " ]]\""
        return transform

    # Reads the most recent points off the maps and puts into the Matrix
    def register_points(self):
        counter = 0
        print "Registering Points"
        for p1, p2 in izip(self.map1.get_points(), self.map2.get_points()):
            if p1 != None and p2 != None:
                # Trianglulate points here
                print p1, p2
                counter += 1
        return counter
    
    # triggered when a point is clicked in either map scene
    def point_handler(self):
        self.update_labels()

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
    usage = "demo.py <Semantic Map Image> <SLAM Map Yaml>"
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