from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import * 
import cv2

class DrawPoint(QGraphicsItem):
    size =  10

    def __init__(self, parent=None):
        super(DrawPoint, self).__init__(parent)
        self.x = -1 - (self.size / 2)
        self.y = -1 - (self.size / 2)
        self.is_drawn = False
        self.setFlag(QGraphicsItem.ItemIsMovable)

    def boundingRect(self):
        return QRectF(self.x, self.y, self.size, self.size)

    def paint(self, painter, option, widget):
        self.is_drawn = True
        pen = QPen(Qt.black)
        pen.setWidth(1)
        brush = QBrush(QColor(128, 128, 255, 128))
        painter.setPen(pen)
        painter.setBrush(brush)
        painter.drawEllipse(self.x, self.y, self.size, self.size)

    def update_pos(self, x, y):
        self.x = x - (self.size / 2)
        self.y = y - (self.size / 2)
        # self.setX(x)
        # self.setY(y)

    def get_pos(self):
        return (self.x + (self.size / 2), self.y + (self.size / 2))
    def getX(self):
        return(self.x + (self.size / 2))
    def getY(self):
        return(self.y + (self.size / 2))



class DrawMap(QGraphicsScene): 
    register = QtCore.pyqtSignal()

    def __init__(self, image, parent=None):
        super(QGraphicsScene, self).__init__(parent)
        self.local_image = QImage(image)

        self.image_format = self.local_image.format()
        self.pixMapItem = QGraphicsPixmapItem(QPixmap(self.local_image), None, self)

        self.pixMapItem.mousePressEvent = self.pixelSelect
        self.position = QPoint(-1, -1)
        self.edit_mode = 1
        self.allPoints = False
        self.markerIndex = 0
        self.canSave = False
        self.line = []
        self.marker = []
        for i in range(0, 4):
            self.marker.append(DrawPoint())
            self.line.append(QGraphicsLineItem())


    # Updates the positin and draws a circle around it
    def pixelSelect( self, event ):
        position = QPoint(event.pos().x(),  event.pos().y())
        print "Mouse click position: " + str(position)
        #If not all of the points have been set, continue to loop through them
        # and place a new point when you click
        #If all of the points have been set, simply change the position of the first one again
        if self.allPoints:
            self.canSave = True
            # Draw a circle around the clicked point
            self.marker[self.markerIndex].update_pos(position.x(), position.y())
            print ("Update Position: " + str(self.marker[self.markerIndex].get_pos()))
        else:
            self.marker[self.markerIndex].update_pos(position.x(), position.y())
            self.addItem(self.marker[self.markerIndex])

        if self.markerIndex == 3:
            self.allPoints= True
            self.drawLine(4)
            self.drawLine(3)
            self.drawLine(2)
            self.drawLine(1)
            for i in range(0, 4):
                print str(self.marker[i].get_pos())
            self.markerIndex = 0
        else:
            self.markerIndex += 1
        self.register.emit()
        self.update()

    # Returns the most recent point
    def getPoint(self):
        return self.marker[self.markerIndex].get_pos()

    #Please provide a line number 1-4
    #Find the points of the corresponding markers and draws a line between them.
    def drawLine(self, lineNumber):
        if lineNumber > len(self.marker):
            return
        x1 = self.marker[lineNumber -1].getX()
        y1 = self.marker[lineNumber -1].getY()
        print "First point: " + str(x1) + str(y1)
        if lineNumber <= 3  :     
            x2 = self.marker[lineNumber].getX()
            y2 = self.marker[lineNumber].getY()
        else:
            x2 = self.marker[0].getX()
            y2 = self.marker[0].getY()
        print "Second Point: " + str(x2) + str(y2)
        pen = QPen(Qt.red)
        pen.setWidth(3)
        self.line[lineNumber-1].setLine(x1, y1, x2, y2)
        self.line[lineNumber-1].setPen(pen)
        self.addItem(self.line[lineNumber-1])
    def change_edit_mode(self, mode):
        self.edit_mode = mode

class Zone(QGraphicsPolygonItem):
    name = "New Zone"
    mode = 0
    points = []
    def __init__(self, parent=None):
        super(QGraphicsPolygonItem, self).__init__(parent)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges)