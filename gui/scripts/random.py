#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
from PySide import QtGui, QtCore

class Example(QtGui.QMainWindow):
    def __init__(self):

        super(Example, self).__init__() #Adding the inherited class
        self.initUI()

    def initUI(self):
        # --LAYOUT-- #
#Creating the stop button
        name = QtGui.QPushButton('Name', self)
        name.move(20, 20)
        name.clicked.connect(self.showDialog)

        col=QtGui.QColor(0, 0, 0)

        self.frame = QtGui.QFrame(self)
        self.frame.setStyleSheet("QWidget { background-color: %s }" %col.name())
        self.frame.setGeometry(130, 22, 100, 100)


        stop = QtGui.QPushButton('STOP', self)
        stop.setToolTip('Press this to immediately <b>STOP</b> the robot')
        stop.resize(stop.sizeHint()) #provides a recommended size for the button
        stop.clicked.connect(self.buttonClicked)
        stop.move(30, 50)

        quit = QtGui.QPushButton(QtGui.QIcon('exit.png'),'QUIT', self)
        quit.clicked.connect(self.buttonClicked)
        quit.clicked.connect(self.close)
        quit.move(150,50)
        # Doing slots and stuff


        self.statusBar()

# --OBJECT CREATION-- #
        #Creating status messages
        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        self.setToolTip('Click the arrows or the map to move')

        #Setting up the toolbar for quick image commands
        exitAction = QtGui.QAction(QtGui.QIcon('exit.png'), 'Exit', self)
        exitAction.setShortcut('Ctrl+Q')
        exitAction.triggered.connect(self.close)
        
        #Now for the zoom tool
        zoomAction = QtGui.QAction(QtGui.QIcon('zoomIn.png'), 'Zoom In', self)
        zoomAction.setShortcut('Ctrl+Z')
        zoomAction.triggered.connect(self.buttonClicked)


        #Adding them to the main toolbar
        self.toolbar = self.addToolBar('Exit')
        self.toolbar.addAction(exitAction)
        self.toolbar.addAction(zoomAction)

        self.statusBar()

        #Adding the menu bar at the top
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        fileMenu.addAction(exitAction)

        


       #Setting up the Window
        #self.setGeometry(300,300,300,150)  #Xpos, Ypos, Width, Height
        self.resize(300,300)
        self.center()
        self.setWindowTitle('Remote Nav') # Window Title
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

    def buttonClicked(self):
        sender = self.sender()
        self.statusBar().showMessage(sender.text() + ' was pressed')

    def showDialog(self):
        #text, ok = QtGui.QInputDialog.getText(self,'Input Dialog', 'Enter your name:')
        #if ok:
        #    self.le.setText(str(text))
        col = QtGui.QColorDialog.getColor()
        if col.isValid():
            self.frame.setStyleSheet("QWidget {background-color: %s}" % col.name())

        
def main():
    app = QtGui.QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
