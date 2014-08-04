#!/usr/bin/env python

## Imports
## ^^^^^^^
from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import QDialog
from pointSelect import Ui_Paths
from components import *


class MainWindow(QDialog, Ui_Paths):
	
	namedFlag = False	#Does the zone at the very minimum have a name?
	savedFlag = True
	editFlag = False	#Are we editing an existing zone?

	def __init__(self, parent=None):
		super(QDialog, self).__init__(parent)
		self.setupUi(self)
		self.controller = ZoneHandler()
		self.scene=DrawMap(QImage("maps/lab_pretty.pgm"), self.controller, self)
		self.map_view.setScene(self.scene)

		## Signals and Slots
		## ^^^^^^^
		self.new_zone_btn.clicked.connect(self.createZone)
		self.save_zone_btn.clicked.connect(self.onSaveClick)
		self.edit_zone.currentIndexChanged.connect(self.loadZone)

	def onSaveClick(self):
		self.saveZone(self.editFlag)

	def createZone(self):
		if self.savedFlag:
			if not self.zone_name.isEnabled():
				self.enableUI()
			self.zone_name.clear()
			self.currentZone = Zone()
			self.editFlag = False
			self.savedFlag = False
		elif self.editFlag:
			saveplz = QMessageBox()
			saveplz.setText("Please save your zone before making a new one.")
			saveplz.setIcon(QMessageBox.Warning)
			saveplz.exec_()

	def nameZone(self, name):
		# self.currentZone.name = "BillY"
		if (name == "" or name== None): 
			print "No Name"
			return
		self.currentZone.name = name
		self.namedFlag = True

	def saveZone(self, editing):
		#There are two kinds of save modes. The first mode is for a completely new zone
		self.nameZone(self.zone_name.text())
		if not self.namedFlag:
			return
		self.currentZone.mode = self.privacy_type.currentIndex()
		#Happens if the edit flag is false (not currently editing)
		if not editing:
			self.addtoList()
		else:
			i = self.edit_zone.currentIndex()
			self.updateList(i)
		self.zone_name.clear()
		self.savedFlag = True
		self.disableUI()

	# Sets the current zone to the specified object in the zone list
	# populates the 
	def loadZone(self, index):
		self.currentZone = self.controller.zoneList[index]
		self.zone_name.setText(self.currentZone.name)
		self.privacy_type.setCurrentIndex(self.currentZone.mode)
		# self.scene.addItem(self.controller.zoneList[index])
		self.controller.zoneList[index].drawPoly()
		self.scene.update()
		self.editFlag = True
		self.enableUI()

	#Changes the data of the zoneList at a given index
	#Updates the zone's name, mode, and list of points
	# It also redraws the polygon into the graphics scene
	def updateList(self, index):
		self.currentZone.import_points(self.scene.getPoints())
		self.controller.zoneList[index] = self.currentZone
		self.edit_zone.setItemText(index, self.controller.zoneList[index].name)
		# self.scene.addItem(self.controller.zoneList[i])
		self.controller.zoneList[index].drawPoly()
		self.scene.update()

	# Adds a new zone to the ZoneList and also adds the zone to the edit dropdown
	def addtoList(self):
		self.currentZone.import_points(self.scene.getPoints())
		self.controller.zoneList.append(self.currentZone)
		recentIndex = len(self.controller.zoneList) - 1
		self.edit_zone.addItem(self.controller.zoneList[recentIndex].name)
		self.scene.addItem(self.controller.zoneList[recentIndex])
		self.scene.update()

	#Enables various UI Elements
	def enableUI(self):
		self.edit_zone.setEnabled(True)
		self.zone_name.setEnabled(True)
		self.privacy_type.setEnabled(True)
		self.save_zone_btn.setEnabled(True)
		self.map_view.setEnabled(True)

	#Disables various UI Elements
	def disableUI(self):
		self.save_zone_btn.setEnabled(False)
		self.zone_name.setEnabled(False)
		self.privacy_type.setEnabled(False)
		self.map_view.setEnabled(False)





if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    window = MainWindow()
    window.resize(1000, 500)
    window.show()
    sys.exit(app.exec_())