from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import * 
import cv2
import yaml
import numpy as np
from itertools import izip
from math import floor

# Taken from MapMetaData.py in the remote_nav package
# Class representing MapMetaData from ROS
class MapMetaData(yaml.YAMLObject):
    yaml_tag = u'!MapMetaData'

    def __init__(self, image, resolution, origin, negate, occupied_thresh, free_thresh):
        self.image = image
        self.resolution = resolution
        self.origin = origin
        self.negate = negate
        self.occupied_thresh = occupied_thresh
        self.free_thresh = free_thresh

class RegisteredMapMetaData(yaml.YAMLObject):
    yaml_tag = u'!RegisteredMapMetaData'

    def __init__(self, semantic_map, slam_map, origin, resolution, slam_width, slam_height, semantic_width, semantic_height, semantic_nodes, slam_nodes, semantic_triangles, slam_triangles):
        self.semantic_map = semantic_map
        self.slam_map = slam_map
        self.origin = origin
        self.resolution = resolution
        self.slam_width = slam_width
        self.slam_height = slam_height
        self.semantic_width = semantic_width
        self.semantic_height = semantic_height
        self.semantic_nodes = semantic_nodes
        self.slam_nodes = slam_nodes
        self.semantic_triangles = semantic_triangles
        self.slam_triangles = slam_triangles

# Taken from MapMetaData.py in the remote_nav package
# Converts a yaml file to MapMetaData. returns a MapMetaData object
def yaml_to_meta_data(file_name, yaml_type):
    # Open the file -- no error cehcking here
    fo = open(file_name)

    # Convert yaml -- no error checking here either
    file_text = fo.read()
    meta_data = yaml.load("--- !" + yaml_type + "\n" + file_text)

    fo.close()

    # return MapMetaData object
    return meta_data

# Simple draggable box which is placed on a map and may be deleted.
class DrawPoint(QGraphicsObject):
    size = 10
    delete_me = QtCore.pyqtSignal()

    def __init__(self, color, parent=None):
        super(DrawPoint, self).__init__(parent)
        self.color = color
        self.x = -1 - (self.size / 2)
        self.y = -1 - (self.size / 2)
        self.is_drawn = False
        self.setFlag(QGraphicsItem.ItemIsMovable)
        self.mousePressEvent = self.ask_to_be_deleted

    # Called when clicked. If Right-click, delete it.
    def ask_to_be_deleted(self, event=None):
        if event == None or event.button() == 2:
            self.delete_me.emit()
            self.setEnabled(False)
            self.setVisible(False)

    def boundingRect(self):
        return QRectF(self.x - 1, self.y - 1, self.size + 2, self.size + 2)

    def paint(self, painter, option, widget):
        self.is_drawn = True
        pen = QPen(Qt.black)
        pen.setWidth(1)
        brush = QBrush(self.color)
        painter.setPen(pen)
        painter.setBrush(brush)
        painter.drawRoundedRect(self.x, self.y, self.size, self.size, self.size / 3, self.size / 3)

    # Sets the (x, y) given to be the center of the resutling shape
    def update_pos(self, x, y):
        self.x = x - (self.size / 2)
        self.y = y - (self.size / 2)

    # Returns center-point of marker
    # Returns a tuple (point)
    def get_pos(self):
        return (self.x + (self.size / 2), self.y + (self.size / 2))

# Scene representing the map image
class DrawMap(QGraphicsScene): 
    # Signal indicating the number of points has changed
    register = QtCore.pyqtSignal()

    def __init__(self, image, parent=None):
        super(QGraphicsScene, self).__init__(parent)
        self.local_image = QImage(image)
        self.image_format = self.local_image.format()
        self.pixMapItem = QGraphicsPixmapItem(QPixmap(self.local_image), None, self)

        self.pixMapItem.mousePressEvent = self.pixelSelect
        self.position = QPoint(-1, -1)
        self.points = []
        self.nodes = []

    # Updates the positin and draws a circle around it
    def pixelSelect( self, event ):
        position = QPoint(event.pos().x(),  event.pos().y())
        btn = event.button()
        if btn == 1:
            self.select_pix(position)

    def select_pix(self, position):
        no_empty_space = True
        index = 0
        while no_empty_space and index < len(self.points):
            if self.points[index] == None:
                no_empty_space = False
            index += 1
        if no_empty_space:
            hue = (index * 30) % 360
        else:
            hue = ((index - 1) * 30) % 360
        marker = DrawPoint(QColor.fromHsv(hue, 255, 255, 128))
        marker.update_pos(position.x(), position.y())
        if not marker.is_drawn:
            self.addItem(marker)
        marker.delete_me.connect(self.delete_marker)
        if no_empty_space:
            # print "Adding point to end of list, index: ", len(self.points)
            self.points.append(marker)
        else:
            # print "Adding point to blank space in list, index: ", index - 1
            self.points[index - 1] = marker

        # print self.points
        self.register.emit()
        self.update()

    # Returns the number of points that are not None
    def get_num_points(self):
        num = 0
        for p in self.points:
            if p != None:
                num += 1
        return num

    # Constructs a list of positions corresponding to the clicked points
    # Returns a list of tuples and Nones
    def get_points(self):
        pts = []
        for p in self.points:
            if p != None:
                pts.append(p.get_pos())
            else:
                pts.append(None)
        return pts

    # Called when a point is right-clicked. This removes the point from 
    # the list and also removes Nones from the end.
    def delete_marker(self):
        index = self.points.index(self.sender())
        self.points[index] = None
        # Remove Nones from end of list
        index = len(self.points) - 1
        while self.points[index] == None and len(self.points) > 1:
            self.points.pop()
            index -= 1
        self.register.emit()
        # print self.points

# Simple box that is draggable
class DrawRobot(QGraphicsObject):
    # This is arbitrary and should probably be scalable
    size = 20

    def __init__(self, parent=None):
        super(QGraphicsObject, self).__init__(parent)
        self.setFlag(QGraphicsItem.ItemIsMovable)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges)

        self.setZValue(10)

    def boundingRect(self):
        return QRectF(-1, -1, self.size+2, self.size+2)

    def paint(self, painter, option, widget):
        pen = QPen(Qt.black, 3, Qt.SolidLine)
        brush = QBrush(Qt.cyan)
        painter.setPen(pen)
        painter.setBrush(brush)
        painter.drawRoundedRect(0, 0, self.size, self.size, self.size / 3, self.size / 3)
    
    # Get the center point of the "robot"
    # Returns a tuple (point)
    def get_pos(self):
        return (self.x + (self.size / 2), self.y + (self.size / 2))

# Handles communication between the two maps and the transform between them
class RobotHandler():
    def __init__(self, robot_1, robot_2):
        self.isenabled = False
        self.robot_1 = robot_1
        self.robot_2 = robot_2
        self.trans_1_to_2 = None
        self.trans_2_to_1 = None
        self.ready = False

        # Set up signals for when the robots move
        self.robot_1.xChanged.connect(self.set_robot_2)
        self.robot_1.yChanged.connect(self.set_robot_2)
        self.robot_2.xChanged.connect(self.set_robot_1)
        self.robot_2.yChanged.connect(self.set_robot_1)
        
    # Set the visibility of the robots based on if they are enabled
    def setEnabled(self, enable_state):
        if self.ready:
            self.isenabled = enable_state
            self.robot_1.setPos(0, 0)
            # Set robot 2 position based on robot 1 position
            self.set_robot_2()
            self.robot_1.setVisible(enable_state)
            self.robot_2.setVisible(enable_state)

    # Set the transforms needed to transform between maps. This also sets the 
    # "ready" flag to True.
    def setTransforms(self, map1_points, map2_points):
        # Set up the dictionary 
        self.trans_1_to_2 = {}
        self.trans_2_to_1 = {}
        first_line = True
        self.registered_points = []
        for p1, p2 in izip(map1_points, map2_points):
            if p1 != None and p2 != None:
                self.registered_points.append((p1, p2))
        self.read_ele("semantic")
        self.read_ele("slam")
        # print self.trans_1_to_2
        # print self.trans_2_to_1
        self.ready = True

    def read_ele(self, map_name):
        if map_name == "semantic":
            filename = "register/semantic.1.ele"
        elif map_name == "slam":
            filename = "register/slam.1.ele"
        else:
            print "in read_ele: not a valid filename"
            return
        first_line = True
        with open(filename, 'r') as f:
            for line in f:
                if first_line:
                    # Do nothing
                    first_line = False
                elif '#' not in line:
                    # This line is not a comment
                    s = line.split()
                    # s[1], s[2], s[3] correspond to indicides of triangle
                    p1 = self.registered_points[int(s[1]) - 1]
                    p2 = self.registered_points[int(s[2]) - 1]
                    p3 = self.registered_points[int(s[3]) - 1]
                    semantic_indicides = (p1[0], p2[0], p3[0])
                    slam_indicies = (p1[1], p2[1], p3[1])
                    tri = TrianglePoints(int(s[0]), semantic_indicides, slam_indicies)
                    if map_name == "semantic":
                        self.trans_1_to_2[int(s[0])] = tri
                    elif map_name == "slam":
                        self.trans_2_to_1[int(s[0])] = tri

    # Given a 1-indexed number, return a color (tuple)
    def triangle_to_color(self, triangle):
        color = (0, 255, triangle * 4)
        return color

    # Given a color (tuple), return a 1-indexed number
    # or -1 if no correspondence
    def color_to_triangle(self, color):
        if color[0] == 0 and color[1] == 0 and color[2] == 0:
            return -1
        else:
            return color[2] / 4

    # Convert a position in map 1 to the map 2 frame
    # Returns a tuple (point) or None
    def convert_to_2(self, point):
        if self.ready:
            x = point.x()
            y = point.y()
            semantic_triangles = cv2.imread("register/semantic.png", 1)
            # Check the bounds
            rows, cols, colors = semantic_triangles.shape
            if y > rows:
                y  = rows - 1
            elif y < 0:
                y = 0
            if x > cols:
                x = cols - 1
            elif x < 0:
                x = 0
            px = semantic_triangles[y, x]
            triangle = self.color_to_triangle(px)
            if triangle == -1:
                # No known correspondence
                return None
            else:
                return self.trans_1_to_2[triangle].get_slam_from_semantic((x, y))
        else:
            return None

    # Convert a position in map 2 to the map 1 frame
    # Returns a tuple (point) or None
    def convert_to_1(self, point):
        if self.ready:
            x = point.x()
            y = point.y()
            slam_triangles = cv2.imread("register/slam.png", 1)
            # Check the bounds
            rows, cols, colors = slam_triangles.shape
            if y > rows:
                y  = rows - 1
            elif y < 0:
                y = 0
            if x > cols:
                x = cols - 1
            elif x < 0:
                x = 0
            px = slam_triangles[y, x]
            triangle = self.color_to_triangle(px)
            if triangle == -1:
                # No known correspondence
                return None
            else:
                return self.trans_2_to_1[triangle].get_semantic_from_slam((x, y))
        else:
            return None

    # Callback for when robot 2's position is changed. Set robot 1 accordingly.
    def set_robot_1(self):
        # Disable signals to prevent blowing the stack
        self.robot_1.blockSignals(True)
        point = self.robot_2.pos()
        pos = self.convert_to_1(point)
        if pos != None:
            self.robot_1.setPos(pos[0], pos[1])
        self.robot_1.blockSignals(False)

    # Callback for when robot 1's position is changed. Set robot 2 accordingly.
    def set_robot_2(self):
        # Disable signals to prevent blowing the stack. 
        self.robot_2.blockSignals(True)
        point = self.robot_1.pos()
        pos = self.convert_to_2(point)
        if pos != None:
            self.robot_2.setPos(pos[0], pos[1])
        self.robot_2.blockSignals(False)

class TrianglePoints():
    def __init__(self, id_num, semantic_pts, slam_pts):
        self.id_num = id_num
        self.semantic_pts = semantic_pts
        self.slam_pts = slam_pts

        # Precompute determinates
        self.slam_det = (self.slam_pts[1][1] - self.slam_pts[2][1]) * (self.slam_pts[0][0] - self.slam_pts[2][0])
        self.slam_det += float((self.slam_pts[2][0] - self.slam_pts[1][0]) * (self.slam_pts[0][1] - self.slam_pts[2][1]))
        self.semantic_det = (self.semantic_pts[1][1] - self.semantic_pts[2][1]) * (self.semantic_pts[0][0] - self.semantic_pts[2][0])
        self.semantic_det += float((self.semantic_pts[2][0] - self.semantic_pts[1][0]) * (self.semantic_pts[0][1] - self.semantic_pts[2][1]))

        # thresholds
        self.neg_thresh = -0.1
        self.pos_thresh =  1.1

    # Given a point in the (x, y) frame of the slam map, return the (x, y)
    # coordinates in the semantic frame. If the given point is not in the 
    # triangle, return None
    def get_semantic_from_slam(self, pt):
        slam_1 = self.slam_pts[0]
        slam_2 = self.slam_pts[1]
        slam_3 = self.slam_pts[2]
        lambda_1 = ((slam_2[1] - slam_3[1]) * (pt[0] - slam_3[0]))
        lambda_1 += ((slam_3[0] - slam_2[0]) * (pt[1] - slam_3[1]))
        lambda_1 /= self.slam_det
        if lambda_1 < self.neg_thresh or lambda_1 > self.pos_thresh:
            print lambda_1
            print "pt ", pt, " not in triangle ", self.id_num
            return None
        lambda_2 = ((slam_3[1] - slam_1[1]) * (pt[0] - slam_3[0]))
        lambda_2 += ((slam_1[0] - slam_3[0]) * (pt[1] - slam_3[1]))
        lambda_2 /= self.slam_det
        if lambda_2 < self.neg_thresh or lambda_2 > self.pos_thresh:
            print lambda_2
            print "pt ", pt, " not in triangle ", self.id_num
            return None
        lambda_3 = 1 - lambda_1 - lambda_2
        x = lambda_1 * self.semantic_pts[0][0]
        x += lambda_2 * self.semantic_pts[1][0]
        x += lambda_3 * self.semantic_pts[2][0]
        y = lambda_1 * self.semantic_pts[0][1]
        y += lambda_2 * self.semantic_pts[1][1]
        y += lambda_3 * self.semantic_pts[2][1]
        return (x, y)

    # Given a point in the (x, y) frame of the semantic map, return the (x, y)
    # coordinates in the slam frame
    def get_slam_from_semantic(self, pt):
        semantic_1 = self.semantic_pts[0]
        semantic_2 = self.semantic_pts[1]
        semantic_3 = self.semantic_pts[2]
        lambda_1 = ((semantic_2[1] - semantic_3[1]) * (pt[0] - semantic_3[0]))
        lambda_1 += ((semantic_3[0] - semantic_2[0]) * (pt[1] - semantic_3[1]))
        lambda_1 /= self.semantic_det
        if lambda_1 < self.neg_thresh or lambda_1 > self.pos_thresh:
            print lambda_1
            print "pt ", pt, " not in triangle ", self.id_num
            return None
        lambda_2 = ((semantic_3[1] - semantic_1[1]) * (pt[0] - semantic_3[0]))
        lambda_2 += ((semantic_1[0] - semantic_3[0]) * (pt[1] - semantic_3[1]))
        lambda_2 /= self.semantic_det
        if lambda_2 < self.neg_thresh or lambda_2 > self.pos_thresh:
            print lambda_2
            print "pt ", pt, " not in triangle ", self.id_num
            return None
        lambda_3 = 1 - lambda_1 - lambda_2
        x = lambda_1 * self.slam_pts[0][0]
        x += lambda_2 * self.slam_pts[1][0]
        x += lambda_3 * self.slam_pts[2][0]
        y = lambda_1 * self.slam_pts[0][1]
        y += lambda_2 * self.slam_pts[1][1]
        y += lambda_3 * self.slam_pts[2][1]
        return (x, y)