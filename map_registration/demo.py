import sys, getopt
from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import * 
import numpy as np
from components import * 
from itertools import izip, izip_longest

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
        self.slam_meta_data = yaml_to_meta_data(slam, "MapMetaData")
        self.slam_origin = self.slam_meta_data.origin
        self.slam_res = self.slam_meta_data.resolution
        self.img_2 = path.dirname(slam) + "/" + self.slam_meta_data.image
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
        self.export_ready = False

        #Changes GUI attributes
        self.toggleRobot.setEnabled(False)

        self.map1.addItem(self.robot1)
        self.map2.addItem(self.robot2)

        self.transform_btn.setToolTip("Apply Transform and show the result")
        self.transform_btn.clicked.connect(self.transform_map)

        self.export_btn.setToolTip("Save the transformed map")
        self.export_btn.clicked.connect(self.export_map)
        self.update_labels()

        self.clearAll_btn.setToolTip("Clears all points from  both maps")
        self.clearAll_btn.clicked.connect(self.clear_all)
        self.clearUnmatched_btn.setToolTip("Clears points which do not have a correpsonding point in the other map")
        self.clearUnmatched_btn.clicked.connect(self.clear_unmatched)

        self.loadPoint_btn.setToolTip("Load points into the map")
        self.loadPoint_btn.clicked.connect(self.loadPoints)
        self.exportPoint_btn.setToolTip("Save matching points. Exporting will do this")
        self.exportPoint_btn.clicked.connect(self.savePoints)

        # The signals are emitted after a click in the map window
        self.map1.register.connect(self.update_labels)
        self.map2.register.connect(self.update_labels)

        # Setting up the robot toggled checkbox 
        self.toggleRobot.stateChanged.connect(self.robot_toggle)


        self.exportZone_btn.clicked.connect(self.export_zones)
        self.importZone_btn.clicked.connect(self.import_zones)
        #Data goes to zoneData, which is a text field

    def loadPoints(self):
        fname = QFileDialog.getOpenFileName(self, 'Open File', "", 'YAML Files (*.yaml)')  
        if fname.isEmpty():
            return
        fname = str(fname)
        meta_data = yaml_to_meta_data(fname, "RegisteredMapMetaData")
        slam_nodes = path.dirname(fname) + "/" + meta_data.slam_nodes
        first_line = True
        with open(slam_nodes, 'r') as f:
            for line in f:
                if first_line:
                    # do nothing
                    first_line = False
                elif '#' not in line:
                    # This line is not a comment
                    s = line.split()
                    x = int(s[1])
                    y = int(s[2])
                    p = QPoint(x, y)
                    self.map2.select_pix(p)
        semantic_nodes = path.dirname(fname) + "/" + meta_data.semantic_nodes
        first_line = True
        with open(semantic_nodes, 'r') as f:
            for line in f:
                if first_line:
                    # do nothing
                    first_line = False
                elif '#' not in line:
                    # This line is not a commnet
                    s = line.split()
                    x = int(s[1])
                    y = int(s[2])
                    p = QPoint(x, y)
                    self.map1.select_pix(p)

    def savePoints(self):
        counter = 0
        slam = ""
        semantic = "" 
        for p1, p2 in izip(self.map1.get_points(), self.map2.get_points()):
            if p1 != None and p2 != None:
                counter += 1
                semantic += str(counter) + " " + str(p1[0]) + " " + str(p1[1]) + "\n"
                slam += str(counter) + " " + str(p2[0]) + " " + str(p2[1]) + "\n"
        semantic = str(counter) + " 2 0 1\n" + semantic
        slam = str(counter) + " 2 0 1\n" + slam
        f1 = open('semantic.node', 'w')
        f2 = open('slam.node', 'w')
        f1.write(semantic)
        f2.write(slam)
        f1.close()
        f2.close()

        call(["mv", "semantic.node", "register/"])
        call(["mv", "slam.node", "register/"])

    def clear_all(self):
        for p in self.map1.points:
            if p != None:
                p.ask_to_be_deleted()
        for p in self.map2.points:
            if p != None:
                p.ask_to_be_deleted()

    def clear_unmatched(self):
        for p1, p2 in izip_longest(self.map1.points, self.map2.points):
            print p1, p2
            if p1 == None and p2 != None:
                p2.ask_to_be_deleted()
            if p2 == None and p1 != None:
                p1.ask_to_be_deleted()


    # Updates the labels telling how many points there are
    def update_labels(self):
        self.label_4.setText(str(self.map1.get_num_points()))
        self.label_5.setText(str(self.map2.get_num_points()))

    # Add (or remove) a robot from the scene
    def robot_toggle(self):
        if self.toggleRobot.isChecked():
            self.robot.setTransforms(self.map1.get_points(), self.map2.get_points())
        self.robot.setEnabled(self.toggleRobot.isChecked())

    # Using matching points, view the transformation of the maps
    def transform_map(self):
        self.triangulate()
        # for each triangle:
        #   calculate affine transform using endpoints
        #   Draw just that region morphed
        #   Append to whole image 

        # Check that three pairs have been make
        # if ((-1, -1) not in self.src) and ((-1, -1) not in self.dst):
        #     print "Transforming Maps"
        #     self.transform_array()
        #     # Apply the transform 
        #     if self.transform != None:
        #         src = cv2.imread(self.img_1, 0)
        #         rows, cols = src.shape
        #         output = cv2.warpAffine(src, self.transform, (cols, rows))
        #         self.outputWindow(output)
        #     # cv2.imshow('Output', output)
        # else:
        #     print "Not enough pairs to transform"

    # Saves the values in a yaml file in the current directory
    def export_map(self):
        self.triangulate()

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
        yaml += "semantic_nodes: semantic.1.node\n"
        yaml += "slam_nodes: slam.1.node\n"
        yaml += "semantic_triangles: semantic.1.ele\n"
        yaml += "slam_triangles: slam.1.ele\n"
        f = open('registration.yaml', 'w')
        f.write(yaml)
        f.close()

        call(["mv", "registration.yaml", "register/"])
        call(["cp", self.img_2, "register/"])
        call(["cp", self.img_1, "register/"])

    # Triangulates both maps and writes these to file, along with the 
    # colored triangle images
    def triangulate(self):
        self.savePoints()

        # Triangulate the nodes
        call(["./triangle/triangle", "./register/semantic.node"])
        call(["./triangle/triangle", "./register/slam.node"])

        # Build the triangulated, colored things
        self.color_triangles("slam")
        self.color_triangles("semantic")

        # Remove the extra .node files
        call(["rm", "register/semantic.node"])
        call(["rm", "register/slam.node"])

        self.robot.setTransforms(self.map1.get_points(), self.map2.get_points())
        self.toggleRobot.setEnabled(True)

    # Creates and writes the triangules based on the triangulation
    def color_triangles(self, image):
        if image == "semantic":
            node_file = "register/semantic.1.node"
            ele_file = "register/semantic.1.ele"
            rows, cols = cv2.imread(self.img_1, 0).shape
        elif image == "slam":
            node_file = "register/slam.1.node"
            ele_file = "register/slam.1.ele"
            rows, cols = cv2.imread(self.img_2, 0).shape
        else:
            return

        tri_img = np.zeros((rows, cols,3), np.uint8)
        first_line = True
        # Seed nodes with None since triangles are 1-indexed
        nodes = [None]
        # From node file, construct ordered list of nodes
        with open(node_file, 'r') as f:
            for line in f:
                if first_line:
                    # do nothing
                    first_line = False
                elif '#' not in line:
                    # This line is not a comment
                    s = line.split()
                    x = int(s[1])
                    y = int(s[2])
                    nodes.append((x, y))
        # From the ele file, color triangles
        first_line = True
        with open(ele_file, 'r') as f:
            for line in f:
                if first_line:
                    # Do nothing
                    first_line = False 
                elif '#' not in line:
                    # This line is not a comment
                    s = line.split()
                    v1 = nodes[int(s[1])]
                    v2 = nodes[int(s[2])]
                    v3 = nodes[int(s[3])]
                    pts = np.array([v1, v2, v3], np.int32)
                    pts = pts.reshape((-1,1,2))
                    color = self.robot.triangle_to_color(int(s[0]))
                    cv2.fillPoly(tri_img,[pts],color)
        img_name = image + ".png"
        cv2.imwrite(img_name, tri_img)
        call(["mv", img_name, "register/" + img_name])

    def outputWindow(self, image):
        cv2.imshow('Preview', image)
        # child = MyWindow(image, self)
        # child.show()

    def export_zones(self):
        if self.export_ready:
            filename = QFileDialog.getSaveFileName(self, 'Save As...', '', 'YAML FILES (*.yaml)')
            if not filename.endsWith(".yaml"):
                filename.append(".yaml")
            fout = open(filename, 'w')
            # Do stuff to write to the file here
            converted_points = self.convert_points(self.myYaml)
            yaml.dump(converted_points, fout)
            fout.close()

    def import_zones(self):
        fname = QFileDialog.getOpenFileName(self, 'Open File', "", 'YAML Files (*.yaml)')
        self.file = fname 
        if fname.isEmpty():
            return
        self.export_ready = True
        fin = open(fname, 'r')      
        with fin:
            # self.import_data = fin.read()
            self.myYaml = yaml.safe_load(fin)
            text = ""
            spacer = " \n"
        # For getting data from the YAML file know this
        #'Zone List' is a list of zones. For all of the zones, iterate through them 
            # self.myYaml['ZoneList'][index]
        # Name is simply the name label as a string
        # Mode is an integer for the privacy type.
        # Points is another list of points, with X and Y values
        # Access points by using:
            # self.myYaml['Zone List'][index]['Points'][x/y]
            for i in range(0, len(self.myYaml['Zone List'])):
                text += self.myYaml['Zone List'][i]['Name'] + spacer
                text += self.privacyMode(self.myYaml['Zone List'][i]['Mode']) + spacer
                # Add the individual points
                for j in range(0, len(self.myYaml['Zone List'][i]['Points'])):
                    text += str(self.myYaml['Zone List'][i]['Points'][j]) + spacer
                text += "-----\n"
            #Output to the preview window to make sure it's okay!         
            self.zoneData.setText(str(text))
            #Import stuff from the yaml file here
        fin.close() 

    def convert_points(self, myYaml):
        dst = cv2.imread(self.img_2, 0)
        img_height, img_width = dst.shape

        for zone in myYaml['Zone List']:
            for point in zone['Points']:
                x = int(point['x'])
                y = int(point['y'])
                pt = self.robot.convert_to_2(QPoint(x, y))
                # Convert from slam image frame to the real world
                x = (pt[0] * self.slam_res) + self.slam_origin[0]
                y = self.slam_origin[1] - ((pt[1] - img_height) * self.slam_res)

                point['x'] = x
                point['y'] = y
        return myYaml

    def privacyMode(self, mode):
        if mode == 1:
            return "Private"
        elif mode == 2:
            return "Public"
        else: 
            return "No Filter"


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

    # Make the output directory
    call(["mkdir", "register/"])

    app = QApplication( sys.argv )
    mainWindow = MainWindow(src, dst)
    mainWindow.resize( 1000, 500 )
    mainWindow.show()
    app.exec_()

if __name__ == '__main__':
    main(sys.argv[1:])