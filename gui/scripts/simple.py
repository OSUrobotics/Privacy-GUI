#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
from PySide import QtGui, QtCore

class Example(QtGui.QWidget):
    def __init__(self):

        super(Example, self).__init__() #Adding the inherited class
        self.initUI()

    def initUI(self):
        # --LAYOUT-- #
#Creating the stop button
        stop = QtGui.QPushButton('STOP', self)
        stop.setToolTip('Press this to immediately <b>STOP</b> the robot')
        stop.resize(stop.sizeHint()) #provides a recommended size for the button

        quit = QtGui.QPushButton('QUIT', self)
        quit.clicked.connect(self.close)

        cb = QtGui.QCheckBox('Show title', self)
        cb.move(20, 20)
        cb.toggle()
        cb.stateChanged.connect(self.changeTitle)

        # Doing slots and stuff
        slide = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        slide.setFocusPolicy(QtCore.Qt.NoFocus)
        slide.setGeometry(30, 40, 100, 30)
        slide.valueChanged[int].connect(self.changeValue)

        self.label = QtGui.QLabel(self)
        self.label.setPixmap(QtGui.QPixmap('mute.png'))
        self.label.setGeometry(160, 40, 80, 30)


        hbox = QtGui.QHBoxLayout()
        hbox.addStretch(1)
        hbox.addWidget(quit)
        hbox.addWidget(stop)

        vbox = QtGui.QVBoxLayout()
        vbox.addStretch(1)
        vbox.addWidget(slide)
        vbox.addLayout(hbox)
        
        self.setLayout(vbox)


# --OBJECT CREATION-- #
        #Creating status messages
        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        self.setToolTip('Click the arrows or the map to move')

        """#Setting up the toolbar for quick image commands
        exitAction = QtGui.QAction(QtGui.QIcon('exit.png'), 'Exit', self)
        exitAction.setShortcut('Ctrl+Q')
        exitAction.triggered.connect(self.close)
        
        #Now for the zoom tool
        zoomAction = QtGui.QAction(QtGui.QIcon('zoomIn.png'), 'Zoom In', self)
        zoomAction.setShortcut('Ctrl+Z')
        zoomAction.triggered.connect(self.close)


        #Adding them to the main toolbar
        self.toolbar = self.addToolBar('Exit')
        self.toolbar.addAction(exitAction)
        self.toolbar.addAction(zoomAction)

        self.statusBar()

        #Adding the menu bar at the top
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        fileMenu.addAction(exitAction)
"""
        


       #Setting up the Window
        #self.setGeometry(300,300,300,150)  #Xpos, Ypos, Width, Height
        self.resize(300,300)
        self.center()
        self.setWindowTitle('QtGui.QCheckBox') # Window Title
        self.setWindowIcon(QtGui.QIcon('icon.png')) #Window Icon

        self.show()

# When the application tries to quit, produce an "Are you sure?" alert
    def closeEvent(self, event):
            
        reply = QtGui.QMessageBox.question(self, 'Message',
            "Are you sure to quit?", QtGui.QMessageBox.Yes | 
            QtGui.QMessageBox.No, QtGui.QMessageBox.No)

        if reply == QtGui.QMessageBox.Yes:
            event.accept()
        else:
            event.ignore()

    def keyPressEvent(self, e):
        if e.key() == QtCore.Qt.Key_Escape:
            self.close()

    def center(self):
        qr = self.frameGeometry()  #Make a temporary rectange with the right specs
        cp = QtGui.QDesktopWidget().availableGeometry().center() #Get screen resolution and centerpoint of monitor
        qr.moveCenter(cp) #Moves that temp rectangle to the center of the monitor
        self.move(qr.topLeft()) #line up our window with the corner of that rectangle
    def changeTitle(self, state):
        if state == QtCore.Qt.Checked:
            self.setWindowTitle('Remote Nav')
        else:
            self.setWindowTitle('Unchecked')

    def changeValue(self, value):
        if value == 0:
            self.label.setPixmap(QtGui.QPixmap('mute.png'))
        elif value > 0 and value <= 30:
            self.label.setPixmap(QtGui.QPixmap('min.png'))
        elif value > 30 and value < 80:
            self.label.setPixmap(QtGui.QPixmap('med.png'))
        else:
            self.label.setPixmap(QtGui.QPixmap('max.png'))

def main():
    app = QtGui.QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
