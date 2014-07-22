from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import * 

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
        painter.setPen(self.color)
        painter.drawRoundedRect(self.x, self.y, self.size, self.size, self.size / 3, self.size / 3)

    def update_pos(self, x, y):
        self.x = x - (self.size / 2)
        self.y = y - (self.size / 2)

    def get_pos(self):
        return (self.x + (self.size / 2), self.y + (self.size / 2))

class DrawMap(QGraphicsScene): 
    def __init__(self, image, parent=None):
        super(QGraphicsScene, self).__init__(parent)
        self.local_image = QImage(image)

        self.image_format = self.local_image.format()
        self.pixMapItem = QGraphicsPixmapItem(QPixmap(self.local_image), None, self)

        self.pixMapItem.mousePressEvent = self.pixelSelect
        self.position = QPoint(-1, -1)
        self.edit_mode = 0

        self.marker_1 = DrawPoint(QtCore.Qt.red)
        self.marker_2 = DrawPoint(QtCore.Qt.green)
        self.marker_3 = DrawPoint(QtCore.Qt.blue)
        self.robot = DrawRobot()
        self.register = QtCore.pyqtSignal()

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
    def __init__(self, parent=None):
        super(QGraphicsObject, self).__init__(parent)

class RobotHandler(QGraphicsObject):
    def __init__(self, parent=None):
        super(QGraphicsObject, self).__init__(parent)
