from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import * 
import cv2
import yaml

# Taken from MapMetaData.py in the remote_nav package
class MapMetaData(yaml.YAMLObject):
    yaml_tag = u'!MapMetaData'

    def __init__(self, image, resolution, origin, negate, occupied_thresh, free_thresh):
        self.image = image
        self.resolution = resolution
        self.origin = origin
        self.negate = negate
        self.occupied_thresh = occupied_thresh
        self.free_thresh = free_thresh

def yaml_to_meta_data(file_name):
    # Open the file -- no error cehcking here
    fo = open(file_name)

    # Convert yaml -- no error checking here either
    file_text = fo.read()
    meta_data = yaml.load("--- !MapMetaData \n" + file_text)

    fo.close()

    # return MapMetaData object
    return meta_data

class DrawPoint(QGraphicsItem):
    size = 10

    def __init__(self, color, parent=None):
        super(DrawPoint, self).__init__(parent)
        self.color = color
        self.x = -1 - (self.size / 2)
        self.y = -1 - (self.size / 2)
        self.is_drawn = False

    def boundingRect(self):
        return QRectF(self.x, self.y, self.size, self.size)

    def paint(self, painter, option, widget):
        self.is_drawn = True
        pen = QPen(self.color)
        pen.setWidth(3)
        painter.setPen(pen)
        painter.drawRoundedRect(self.x, self.y, self.size, self.size, self.size / 3, self.size / 3)

    def update_pos(self, x, y):
        self.x = x - (self.size / 2)
        self.y = y - (self.size / 2)

    def get_pos(self):
        return (self.x + (self.size / 2), self.y + (self.size / 2))

class DrawMap(QGraphicsScene): 
    register = QtCore.pyqtSignal()

    def __init__(self, image, parent=None):
        super(QGraphicsScene, self).__init__(parent)
        self.local_image = QImage(image)

        self.image_format = self.local_image.format()
        self.pixMapItem = QGraphicsPixmapItem(QPixmap(self.local_image), None, self)

        self.pixMapItem.mousePressEvent = self.pixelSelect
        self.position = QPoint(-1, -1)
        self.edit_mode = 0

        self.marker_1 = DrawPoint(QtCore.Qt.red)
        self.marker_2 = DrawPoint(QtCore.Qt.darkGreen)
        self.marker_3 = DrawPoint(QtCore.Qt.blue)

    # Updates the positin and draws a circle around it
    def pixelSelect( self, event ):
        position = QPoint(event.pos().x(),  event.pos().y())
        marker = None
        if self.edit_mode == 1:
            marker = self.marker_1
        elif self.edit_mode == 2:
            marker = self.marker_2
        elif self.edit_mode == 3:
            marker = self.marker_3

        if marker != None:
            # Draw a circle around the clicked point
            marker.update_pos(position.x(), position.y())
            if not marker.is_drawn:
                self.addItem(marker)
            self.register.emit()
            self.update()

    # Returns the most recent point
    def getPoint(self):
        if self.edit_mode == 1:
            return self.marker_1.get_pos()
        elif self.edit_mode == 2:
            return self.marker_2.get_pos()
        elif self.edit_mode == 3:
            return self.marker_3.get_pos()
        else:
            return (-1, -1)

    def change_edit_mode(self, mode):
        self.edit_mode = mode

class DrawRobot(QGraphicsObject):
    size = 20

    def __init__(self, parent=None):
        super(QGraphicsObject, self).__init__(parent)
        self.setFlag(QGraphicsItem.ItemIsMovable)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges)

        self.setZValue(10)

    def boundingRect(self):
        return QRectF(0, 0, self.size, self.size)

    def paint(self, painter, option, widget):
        pen = QPen(Qt.black, 3, Qt.SolidLine)
        brush = QBrush(Qt.cyan)
        painter.setPen(pen)
        painter.setBrush(brush)
        painter.drawRoundedRect(0, 0, self.size, self.size, self.size / 3, self.size / 3)
    def get_pos(self):
        return (self.x + (self.size / 2), self.y + (self.size / 2))
class RobotHandler():
    def __init__(self, robot_1, robot_2):
        self.isenabled = False
        self.robot_1 = robot_1
        self.robot_2 = robot_2
        self.trans_1_to_2 = None
        self.trans_2_to_1 = None
        self.ready = False

        self.robot_1.xChanged.connect(self.set_robot_2)
        self.robot_1.yChanged.connect(self.set_robot_2)
        self.robot_2.xChanged.connect(self.set_robot_1)
        self.robot_2.yChanged.connect(self.set_robot_1)
        
    def setEnabled(self, enable_state):
        if self.ready:
            self.isenabled = enable_state
            self.robot_1.setPos(0, 0)
            self.set_robot_2()
            self.robot_1.setVisible(enable_state)
            self.robot_2.setVisible(enable_state)

    def setTransforms(self, src, dst):
        self.trans_1_to_2 = cv2.getAffineTransform(src, dst)
        print "Transform from 1 to 2:", self.trans_1_to_2
        self.trans_2_to_1 = cv2.getAffineTransform(dst, src)
        print "Transform from 2 to 1:", self.trans_2_to_1
        self.ready = True
        return self.trans_1_to_2

    def convert_to_2(self, point):
        if self.ready:
            x = point.x()
            y = point.y()
            x_prime = (self.trans_1_to_2[0][0] * x) + (self.trans_1_to_2[0][1] * y) + self.trans_1_to_2[0][2]
            y_prime = (self.trans_1_to_2[1][0] * x) + (self.trans_1_to_2[1][1] * y) + self.trans_1_to_2[1][2]
            return (x_prime, y_prime)
        else:
            return None

    def convert_to_1(self, point):
        if self.ready:
            x = point.x()
            y = point.y()
            x_prime = (self.trans_2_to_1[0][0] * x) + (self.trans_2_to_1[0][1] * y) + self.trans_2_to_1[0][2]
            y_prime = (self.trans_2_to_1[1][0] * x) + (self.trans_2_to_1[1][1] * y) + self.trans_2_to_1[1][2]
            return (x_prime, y_prime)
        else:
            return None

    def set_robot_1(self):
        self.robot_1.blockSignals(True)
        point = self.robot_2.pos()
        pos = self.convert_to_1(point)
        self.robot_1.setPos(pos[0], pos[1])
        self.robot_1.blockSignals(False)

    def set_robot_2(self):
        self.robot_2.blockSignals(True)
        point = self.robot_1.pos()
        pos = self.convert_to_2(point)
        self.robot_2.setPos(pos[0], pos[1])
        self.robot_2.blockSignals(False)