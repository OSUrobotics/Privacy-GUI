import sys, getopt
from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import * 
import cv2
import numpy as np

class MainWindow(QWidget):
    def __init__(self, map1, map2, parent=None):
        super(QWidget, self).__init__(parent)

        self.setWindowTitle('Main Window')

        layout = QVBoxLayout()
        mapLayout = QHBoxLayout()
        buttonLayout = QHBoxLayout()
 
        # Sets up the maps
        self.source = QGraphicsView()
        self.destination = QGraphicsView()
        self.map1 = DrawMap(map1, self)
        self.source.setScene( self.map1 )
        self.map2 = DrawMap(map2, self)
        self.destination.setScene( self.map2 )
        mapLayout.addWidget(self.source)
        mapLayout.addWidget(self.destination)

        # BUTTONS
        register_btn = QtGui.QPushButton('Register Points', self)
        register_btn.setToolTip("Pair the points currently selected in the map")
        register_btn.clicked.connect(self.register_points)
        register_btn.resize(register_btn.sizeHint())
        buttonLayout.addWidget(register_btn)

        warp_btn = QtGui.QPushButton('Apply Transform', self)
        warp_btn.setToolTip("Apply Affine Transform defined by the registered points")
        warp_btn.clicked.connect(self.transform_map)
        warp_btn.resize(warp_btn.sizeHint())
        buttonLayout.addWidget(warp_btn)

        layout.addLayout(mapLayout)
        layout.addLayout(buttonLayout)
        self.setLayout( layout )

        # Set up variables for point registration and transformation
        self.src = []
        self.dst = []

    # Using registred points, transform the maps
    def transform_map(self):
        # Check that three pairs have been make
        if len(self.src) == 3:
            print "Transforming Maps"
            # Turn the pairs into an Affine Transformation matrix
            numpy_src = np.array(self.src, dtype='float32')
            numpy_dst = np.array(self.dst, dtype='float32')
            transform = cv2.getAffineTransform(numpy_src, numpy_dst)
            print "Transform: ", transform
            # Apply the transform 
            src = cv2.imread('lab.pgm', 0)
            rows, cols = src.shape
            output = cv2.warpAffine(src, transform, (cols, rows))
            cv2.imshow('Output', output)
        else:
            print "Not enough pairs to transform"

    # Reads the most recent points off the maps and puts into the Matrix
    def register_points(self):
        # check that both map1.getPoint() and map2.getPoint() have been set
        if (self.map1.getPoint() != (-1, -1)) and (self.map2.getPoint() != (-1, -1)):
            if (self.map1.getPoint() not in self.src) and (self.map2.getPoint() not in self.dst):
                print "Registering Points"
                self.src.append(self.map1.getPoint())
                self.dst.append(self.map2.getPoint())
                if len(self.src) > 3:
                    self.src.pop(0)
                    self.dst.pop(0)
                print "Source: ", self.src
                print "Destination: ", self.dst
            else:
                print "Cannot pair new point with old point"
        else:
            print "Not all points have been set"

class DrawPoint(QGraphicsItem):
    size = 10

    def __init__(self, parent=None):
        super(DrawPoint, self).__init__(parent)
        self.x = 0
        self.y = 0

    def boundingRect(self):
        penWidth = 1.0
        return QRectF(self.x, self.y, self.size, self.size)

    def paint(self, painter, option, widget):
        painter.drawRoundedRect(self.x, self.y, self.size, self.size, self.size / 2, self.size / 2)

    def update_pos(self, x, y):
        self.x = x - (self.size / 2)
        self.y = y - (self.size / 2)

class DrawMap(QGraphicsScene): 
    def __init__(self, image, parent=None):
        super(QGraphicsScene, self).__init__(parent)
        self.local_image = QImage(image)

        self.image_format = self.local_image.format()
        self.pixMapItem = QGraphicsPixmapItem(QPixmap(self.local_image), None, self)

        self.pixMapItem.mousePressEvent = self.pixelSelect
        self.marker = DrawPoint()
        self.is_drawn = False
        self.position = QPoint(-1, -1)

    # Updates the positin and draws a circle around it
    def pixelSelect( self, event ):
        self.position = QPoint(event.pos().x(),  event.pos().y())
        # Draw a circle around the clicked point
        color = QColor.fromRgb(self.local_image.pixel( self.position ) )
        # Draw a circle around the clicked point
        self.marker.update_pos(self.position.x(), self.position.y())
        if not self.is_drawn:
            self.addItem(self.marker)
            self.is_drawn = True
        self.update()

    # Returns the most recent point
    def getPoint(self):
        point = (self.position.x(), self.position.y())
        return point

def main(argv):
    usage = "demo.py -s <source image> -o <destination image>"
    src = ""
    dst = ""

    try:
        opts, args = getopt.getopt(argv, "hs:d:",["src=","dst="])
    except getoptError:
        print usage
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print usage
            sys.exit()
        elif opt in ("-s", "--src"):
            src = arg
        elif opt in ("-d", "--dst"):
            dst = arg

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