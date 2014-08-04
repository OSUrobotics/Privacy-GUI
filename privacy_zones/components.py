from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import * 
import cv2


#Creates a point at the clicked location and stores the coordinates of the center of that point
class DrawPoint(QGraphicsObject):
    #The size of the circle drawn
    size =  10
    def __init__(self, parent=None):
        super(DrawPoint, self).__init__(parent)
        self.x = -1 - (self.size / 2)
        self.y = -1 - (self.size / 2)
        #Prevents the item from being drawn multiple times in a scene.
        self.is_drawn = False

        # self.xChanged.connect(self.update_self)
        # self.yChanged.connect(self.update_pos)

    #Required to be defined by any QGraphicsObject class
    def boundingRect(self):
        return QRectF(self.x, self.y, self.size, self.size)
    #Also required
    def paint(self, painter, option, widget):
        self.is_drawn = True
        pen = QPen(Qt.black)
        pen.setWidth(1)
        brush = QBrush(QColor(128, 128, 255, 128))
        painter.setPen(pen)
        painter.setBrush(brush)
        painter.drawEllipse(self.x, self.y, self.size, self.size)
    
    #Updates its X an Y position. Used in a signal,slot mechanism
    def update_self(self):
        self.blockSignals(True)
        self.x = self.x()
        self.y = self.y()
        self.blockSignals(False)

    #Updates its X and Y position to a given x/y (remapped to the center)
    def update_pos(self, x, y):
        self.x = x - (self.size / 2)
        self.y = y - (self.size / 2)
        # self.setX(x)
        # self.setY(y)

        #Returns the position as a tuple
    def get_pos(self):
        return (self.x + (self.size / 2), self.y + (self.size / 2))
    #Returns only the X Position at the center line
    def getX(self):
        return(self.x + (self.size / 2))
    # Returns only the Y position at the center line
    def getY(self):
        return(self.y + (self.size / 2))


#The Graphics Scene that holds all of the objects added to it. Registers click events to send to other objects
# Constructed with a map image that all other items are placed over.
class DrawMap(QGraphicsScene): 
    register = QtCore.pyqtSignal()

    def __init__(self, image, parent=None):
        super(QGraphicsScene, self).__init__(parent)
        self.local_image = QImage(image)
        self.image_format = self.local_image.format()
        self.pixMapItem = QGraphicsPixmapItem(QPixmap(self.local_image), None, self)

        #When the mouse is pressed, select and draw a new marker
        self.pixMapItem.mousePressEvent = self.pixelSelect
        # The position clicked
        self.position = QPoint(-1, -1)
        #Unused at the moment
        self.edit_mode = 1

        #The zone handler handles the polygon objects created in the graphics view
        self.controller = ZoneHandler()



    # Updates the position and draws a circle around it
    def pixelSelect( self, event ):
        position = QPoint(event.pos().x(),  event.pos().y())
        print "Mouse click position: " + str(position)
        #If not all of the points have been set, continue to loop through them
        # and place a new point when you click
        #If all of the points have been set, simply change the position of the first one again
        self.controller.setNextMarker(position)
        if not self.controller.getMarker().is_drawn:
            self.addItem(self.controller.getMarker())
        if self.controller.allPoints:
            for i in range(0, len(self.controller.line)):
                if not self.controller.lineDrawn[i]:
                    self.addItem(self.controller.getLine(i))
                else:
                    self.controller.redraw(i+1)
        self.register.emit()
        #Make sure to update the scene with the new drawings.
        self.update()

    # Returns the points stored in the controller as a list
    def getPoints(self):
        temp = []
        for i in range(0, len(self.controller.marker)):
            temp.append(self.controller.marker[i].get_pos())
        return temp

    #Removes all existing objects excluding the background map    
    def clear(self):
        pass

    #Takes a polygon (usually passed in from the corresponding zone) and adds it to the scene
    def drawPoly(self, polygon):
        temp = QGraphicsPolygonItem()
        temp.copy(polygon)
        self.addItem(temp)


    def change_edit_mode(self, mode):
        self.edit_mode = mode

#The Zone Handler class handles the visual markers for the zone (not the zones themselves)
#It is used to place lines and markers into the scene
class ZoneHandler():
    def __init__(self, parent=None):
        self.allPoints = False #Have all points been placed
        self.canSave = False #Can the zone be saved?
        self.marker = [] #List of DrawPoint objects
        self.line = [] #List of QGraphicsLineItems
        self.lineDrawn = [] #Boolean list of if a corresponding line was drawn
        self.markerIndex = 0 #Index of the current marker

        #Initialize the marker and line arrays to be the proper size
        for i in range(0, 4):
            self.marker.append(DrawPoint())
            self.line.append(QGraphicsLineItem())
            self.lineDrawn.append(False)

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
        # self.addItem(self.line[lineNumber-1])
    def redraw(self, lineNumber):
        self.drawLine(lineNumber-1)

    #Draws the next marker in the list and increments the marker index
    #Only needs the position ()
    def setNextMarker(self, position):
            # Updates the position and draws a circle around it

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
            #Output the marker object of a given index

        if self.markerIndex == 3:
            self.allPoints= True
            for i in range(1, 5):
                self.drawLine(i)

            for i in range(0, 4):
                print str(self.marker[i].get_pos())
            self.markerIndex = 0
        else:
            self.markerIndex += 1
    #Returns a marker object from the list at a given index
    def getMarker(self):
        return self.marker[self.markerIndex]

    #Returns a line object from the list at a given index
    def getLine(self, lineNumber):
        self.lineDrawn[lineNumber] = True
        return self.line[lineNumber]

class Zone():
    name = "New Zone"
    mode = 0
    points = []
    poly = QGraphicsPolygonItem()
    def __init__(self, parent=None):
        pass

    def import_points(self, pointList):
        self.points = pointList
        self.points.append(pointList[0]) #Add the first point twice to complete the shape
        for i in range (0, len(self.points)):
            print str(i) + str(self.points[i])
    #Draws the polygon given the points that were previously imported.         
    def drawPoly(self):
        if self.points == None:
            return
        temp = QPolygonF()

        for i in range(0, len(self.points)):
            qpoint = QPointF(self.points[i][0], self.points[i][1])
            temp << qpoint
        self.poly.setPolygon(temp)
        return temp
