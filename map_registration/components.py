from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import * 
import cv2
import yaml

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

# Taken from MapMetaData.py in the remote_nav package
# Converts a yaml file to MapMetaData. returns a MapMetaData object
def yaml_to_meta_data(file_name):
    # Open the file -- no error cehcking here
    fo = open(file_name)

    # Convert yaml -- no error checking here either
    file_text = fo.read()
    meta_data = yaml.load("--- !MapMetaData \n" + file_text)

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
    def ask_to_be_deleted(self, event):
        if event.button() == 2:
            self.delete_me.emit()
            self.setEnabled(False)
            self.setVisible(False)

    def boundingRect(self):
        return QRectF(self.x, self.y, self.size, self.size)

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
        return QRectF(0, 0, self.size+2, self.size+2)

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
    def setTransforms(self):
        self.ready = True

    # Convert a position in map 1 to the map 2 frame
    # Returns a tutle (point) or None
    def convert_to_2(self, point):
        if self.ready:
            x = point.x()
            y = point.y()
            x_prime = (self.trans_1_to_2[0][0] * x) + (self.trans_1_to_2[0][1] * y) + self.trans_1_to_2[0][2]
            y_prime = (self.trans_1_to_2[1][0] * x) + (self.trans_1_to_2[1][1] * y) + self.trans_1_to_2[1][2]
            return (x_prime, y_prime)
        else:
            return None

    # Convert a position in map 2 to the map 1 frame
    # Returns a tuple (point) or None
    def convert_to_1(self, point):
        if self.ready:
            x = point.x()
            y = point.y()
            x_prime = (self.trans_2_to_1[0][0] * x) + (self.trans_2_to_1[0][1] * y) + self.trans_2_to_1[0][2]
            y_prime = (self.trans_2_to_1[1][0] * x) + (self.trans_2_to_1[1][1] * y) + self.trans_2_to_1[1][2]
            return (x_prime, y_prime)
        else:
            return None

    # Callback for when robot 2's position is changed. Set robot 1 accordingly.
    def set_robot_1(self):
        # Disable signals to prevent blowing the stack
        self.robot_1.blockSignals(True)
        point = self.robot_2.pos()
        pos = self.convert_to_1(point)
        self.robot_1.setPos(pos[0], pos[1])
        self.robot_1.blockSignals(False)

    # Callback for when robot 1's position is changed. Set robot 2 accordingly.
    def set_robot_2(self):
        # Disable signals to prevent blowing the stack. 
        self.robot_2.blockSignals(True)
        point = self.robot_1.pos()
        pos = self.convert_to_2(point)
        self.robot_2.setPos(pos[0], pos[1])
        self.robot_2.blockSignals(False)