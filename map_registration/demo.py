import sys
from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import * 

class MainWindow(QWidget):
    def __init__(self, parent=None):
        super(QWidget, self).__init__(parent)

        self.setWindowTitle('Main Window')
        self.source = QGraphicsView()
        self.destination = QGraphicsView()

        layout = QVBoxLayout()
        mapLayout = QHBoxLayout()
        buttonLayout = QHBoxLayout()
 

        self.map1 = DrawMap('lab.pgm', self)
        self.source.setScene( self.map1 )
        self.map2 = DrawMap('lab_pretty.pgm', self)
        self.destination.setScene( self.map2 )
        mapLayout.addWidget(self.source)
        mapLayout.addWidget(self.destination)

        # self.map1 = DrawMap('lab.pgm', 'source', self)
        # self.map1.show()
        # self.map2 = DrawMap('lab_pretty.pgm', 'destination', self)
        # self.map2.show()




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

    # Using registred points, transform the maps
    def transform_map(self):
        print "Transforming Maps"
        # Check that three pairs have been make
        # Turn the pairs into an Affine Transformation matrix
        # Apply the transform 

    # Reads the most recent points off the maps and puts into the Matrix
    def register_points(self):
        print "Registering Points"
        # check that both map1.position and map2.position have been set
        # If yes, put them into row i of matrixy thing
        # Increment (mod 3) i

class DrawPoint(QGraphicsItem):
    def __init__(self, parent=None):
        super(DrawPoint, self).__init__(parent)
        self.x = 0
        self.y = 0

    def boundingRect(self):
        penWidth = 1.0
        return QRectF(self.x, self.y, 20, 20)

    def paint(self, painter, option, widget):
        painter.drawRoundedRect(self.x, self.y, 20, 20, 5, 5)

    def update_pos(self, x, y):
        self.x = x - 10
        self.y = y - 10
        # Force a re-paint

class DrawMap(QGraphicsScene): 
    def __init__(self, image, parent=None):
        super(QGraphicsScene, self).__init__(parent)
        self.local_image = QImage(image)

        self.image_format = self.local_image.format()
        self.pixMapItem = QGraphicsPixmapItem(QPixmap(self.local_image), None, self)

        self.pixMapItem.mousePressEvent = self.pixelSelect
        self.marker = DrawPoint()
        self.is_drawn = False

    def closeEvent(self, event):
        event.accept()

    # Updates the positin and draws a circle around it
    def pixelSelect( self, event ):
        self.position = QPoint(event.pos().x(),  event.pos().y())
        # Draw a circle around the clicked point
        self.marker.update_pos(self.position.x(), self.position.y())
        if not self.is_drawn:
            self.addItem(self.marker)
            self.is_drawn = True
        self.update()

    # Returns the most recent point
    def getPoint(self):
        return self.position

def main():
    app = QApplication( sys.argv )
    mainWindow = MainWindow()
    mainWindow.resize( 1000, 500 )
    mainWindow.show()
    app.exec_()

if __name__ == '__main__':
    main()